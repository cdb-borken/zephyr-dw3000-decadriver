/*
 * Copyright 2015 (c) DecaWave Ltd, Dublin, Ireland.
 * Copyright 2019 (c) Frederic Mes, RTLOC.
 * Copyright 2021 (c) Callender-Consulting LLC.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>

#include <hal/nrf_gpio.h>
#include <hal/nrf_spim.h>

#include <string.h>

#include "dw3000_spi.h"

#include "version.h"

/* This file implements the SPI functions required by decadriver */

LOG_MODULE_DECLARE(dw3000, CONFIG_DW3000_LOG_LEVEL);

#define DW_INST DT_INST(0, decawave_dw3000)
#define DW_SPI DT_PARENT(DT_INST(0, decawave_dw3000))

#if !DT_NODE_HAS_COMPAT(DW_SPI, nordic_nrf_spim)
#error "DW3000 SPI requires a nordic,nrf-spim controller"
#endif

/* Transfers are issued straight to the SPIM registers. Going through the Zephyr
 * SPI API costs ~22us per access (semaphores, nrfx setup, an interrupt and a
 * transaction per spi_buf) against ~1.3us of actual clocking at 32MHz. Zephyr's
 * driver is still used once at init to apply pinctrl and configure the
 * peripheral. */
static NRF_SPIM_Type* const spim = (NRF_SPIM_Type*)DT_REG_ADDR(DW_SPI);

#define DW_CS_PSEL (DT_PROP(DT_SPI_DEV_CS_GPIOS_CTLR(DW_INST), port) * 32U + DT_SPI_DEV_CS_GPIOS_PIN(DW_INST))

BUILD_ASSERT(DT_SPI_DEV_CS_GPIOS_FLAGS(DW_INST) & GPIO_ACTIVE_LOW, "DW3000 chip select must be active low");

/* Coalescing header and payload into one buffer keeps a register access to a
 * single hardware transaction. Larger accesses are split, holding CS asserted. */
#define SPI_SCRATCH_LEN 256

/* Bound on the polled wait; only reached if the transfer never completes. */
#define SPI_XFER_MAX_SPINS 1000000UL

static uint8_t tx_scratch[SPI_SCRATCH_LEN] __aligned(4);
static uint8_t rx_scratch[SPI_SCRATCH_LEN] __aligned(4);

/* Preserves the serialisation the Zephyr SPI API used to provide, since
 * dwt_isr() runs on the system workqueue alongside the ranging thread. */
static K_SEM_DEFINE(spi_lock, 1, 1);

static const struct device* spi;
#if KERNEL_VERSION_MAJOR > 3 || (KERNEL_VERSION_MAJOR == 3 && KERNEL_VERSION_MINOR >= 4)
static struct spi_cs_control cs_ctrl = SPI_CS_CONTROL_INIT(DW_INST, 0);
#else
static struct spi_cs_control* cs_ctrl = SPI_CS_CONTROL_PTR_DT(DW_INST, 0);
#endif
static struct spi_config spi_cfgs[2] = {0}; // configs for slow and fast
static struct spi_config* spi_cfg;

static nrf_spim_frequency_t spim_freq_of(uint32_t hz)
{
    if (hz >= MHZ(32))
    {
        return NRF_SPIM_FREQ_32M;
    }
    else if (hz >= MHZ(16))
    {
        return NRF_SPIM_FREQ_16M;
    }
    else if (hz >= MHZ(8))
    {
        return NRF_SPIM_FREQ_8M;
    }
    else if (hz >= MHZ(4))
    {
        return NRF_SPIM_FREQ_4M;
    }
    else if (hz >= MHZ(2))
    {
        return NRF_SPIM_FREQ_2M;
    }
    else
    {
        return NRF_SPIM_FREQ_1M;
    }
}

static inline void cs_assert(void)
{
    nrf_gpio_pin_clear(DW_CS_PSEL);
}

static inline void cs_deassert(void)
{
    nrf_gpio_pin_set(DW_CS_PSEL);
}

#ifdef CONFIG_NRF52_ANOMALY_198_WORKAROUND

#define AHB_SLAVE_PRIO_REG 0x40000E00UL

static uint32_t anomaly_198_saved;

/* SPIM3 corrupts TX data when EasyDMA and the CPU contend for the same RAM
 * block; mirrors the arbitration fix nrfx applies around every transfer. */
static void anomaly_198_enter(const uint8_t* buf, size_t len)
{
    anomaly_198_saved = *(volatile uint32_t*)AHB_SLAVE_PRIO_REG;

    if (len == 0)
    {
        return;
    }

    uint32_t end_addr = (uint32_t)buf + len;
    uint32_t block_addr = (uint32_t)buf & ~0x1FFFUL;
    uint32_t block_flag = 1UL << ((block_addr >> 13) & 0xFFFF);
    uint32_t occupied = 0;

    if (block_addr >= 0x20010000UL)
    {
        occupied = 1UL << 8;
    }
    else
    {
        do
        {
            occupied |= block_flag;
            block_flag <<= 1;
            block_addr += 0x2000;
        } while (block_addr < end_addr && block_addr < 0x20012000UL);
    }

    *(volatile uint32_t*)AHB_SLAVE_PRIO_REG = occupied;
}

static void anomaly_198_exit(void)
{
    *(volatile uint32_t*)AHB_SLAVE_PRIO_REG = anomaly_198_saved;
}

#else
#define anomaly_198_enter(buf, len) ((void)0)
#define anomaly_198_exit() ((void)0)
#endif /* CONFIG_NRF52_ANOMALY_198_WORKAROUND */

/* Runs one SPIM transaction without touching CS. TX and RX lengths may differ;
 * the shorter side is padded with (for TX) or discards (for RX) the overrun
 * character. */
static int spim_run(const uint8_t* tx, size_t tx_len, uint8_t* rx, size_t rx_len)
{
    int ret = 0;

    anomaly_198_enter(tx, tx_len);

    spim->TXD.PTR = (uint32_t)tx;
    spim->TXD.MAXCNT = tx_len;
    spim->RXD.PTR = (uint32_t)rx;
    spim->RXD.MAXCNT = rx_len;

    spim->EVENTS_END = 0;
    (void)spim->EVENTS_END;
    spim->TASKS_START = 1;

    uint32_t spins = 0;
    while (spim->EVENTS_END == 0)
    {
        if (++spins > SPI_XFER_MAX_SPINS)
        {
            LOG_ERR("SPI transfer timed out");
            ret = -ETIMEDOUT;
            break;
        }
    }
    spim->EVENTS_END = 0;
    (void)spim->EVENTS_END;

    anomaly_198_exit();

    return ret;
}

int dw3000_spi_init(void)
{
    /* set common SPI config */
    for (int i = 0; i < ARRAY_SIZE(spi_cfgs); i++)
    {
        spi_cfgs[i].cs = cs_ctrl;
        spi_cfgs[i].operation = SPI_WORD_SET(8);
    }

    /* SPI clock speed: Slow 2MHz, Max clock speed from DTS
     * We have to keep two different config structures due to the way the SPI
     * driver works */
    spi_cfgs[0].frequency = 2000000;
    spi_cfgs[1].frequency = DT_PROP(DW_INST, spi_max_frequency);
    spi_cfg = &spi_cfgs[0];

    spi = DEVICE_DT_GET(DW_SPI);
    if (!spi)
    {
        LOG_ERR("DW3000 SPI binding failed");
        return -1;
    }
    else
    {
        LOG_INF("DW3000 SPI (max %dMHz)", spi_cfgs[1].frequency / 1000000);
    }

#if CONFIG_PM_DEVICE
    enum pm_device_state pstate;
    int rc = pm_device_state_get(spi, &pstate);
    if (rc)
    {
        LOG_ERR("PM state get %d", rc);
    }

    if (pstate != PM_DEVICE_STATE_ACTIVE)
    {
        rc = pm_device_action_run(spi, PM_DEVICE_ACTION_RESUME);
        if (rc)
        {
            LOG_ERR("PM resume %d", rc);
        }
    }
#endif

    // initialized correctly at boot but after fini we need to reconfigure
    gpio_pin_configure_dt(&spi_cfg->cs.gpio, GPIO_OUTPUT_HIGH);

    /* One transfer through the Zephyr driver so it applies pinctrl and runs
     * nrfx_spim_init(); afterwards the peripheral stays configured and we drive
     * it directly, so its interrupt must be silenced. */
    uint8_t dummy = 0;
    const struct spi_buf buf = {.buf = &dummy, .len = 1};
    const struct spi_buf_set set = {.buffers = &buf, .count = 1};

    spi_cfg = &spi_cfgs[0];
    int err = spi_transceive(spi, spi_cfg, &set, NULL);
    if (err)
    {
        LOG_ERR("DW3000 SPI configure failed: %d", err);
        return err;
    }

    spim->INTENCLR = UINT32_MAX;
    spim->ORC = 0xFF;
    /* nrfx disables the peripheral again at the end of every transfer, so it
     * must be re-enabled now that we drive TASKS_START ourselves. */
    nrf_spim_enable(spim);
    dw3000_spi_speed_slow();

    return 0;
}

void dw3000_spi_speed_slow(void)
{
    spi_cfg = &spi_cfgs[0];
    nrf_spim_frequency_set(spim, spim_freq_of(spi_cfgs[0].frequency));
}

void dw3000_spi_speed_fast(void)
{
    spi_cfg = &spi_cfgs[1];
    nrf_spim_frequency_set(spim, spim_freq_of(spi_cfgs[1].frequency));
}

void dw3000_spi_fini(void)
{
    // TODO: I can't find a SPI uninit function in Zephyr
#if CONFIG_PM_DEVICE
    int rc = pm_device_action_run(spi, PM_DEVICE_ACTION_SUSPEND);
    if (rc)
    {
        LOG_ERR("PM FINI suspend %d", rc);
    }
#endif
    gpio_pin_configure_dt(&spi_cfg->cs.gpio, GPIO_DISCONNECTED);
}

int32_t dw3000_spi_write_crc(uint16_t headerLength, const uint8_t* headerBuffer, uint16_t bodyLength, const uint8_t* bodyBuffer, uint8_t crc8)
{
    const size_t total = (size_t)headerLength + bodyLength + 1U;
    int ret;

    k_sem_take(&spi_lock, K_FOREVER);
    cs_assert();

    if (total <= SPI_SCRATCH_LEN)
    {
        memcpy(tx_scratch, headerBuffer, headerLength);
        if (bodyLength)
        {
            memcpy(&tx_scratch[headerLength], bodyBuffer, bodyLength);
        }
        tx_scratch[headerLength + bodyLength] = crc8;

        ret = spim_run(tx_scratch, total, NULL, 0);
    }
    else
    {
        ret = spim_run(headerBuffer, headerLength, NULL, 0);
        if (ret == 0)
        {
            ret = spim_run(bodyBuffer, bodyLength, NULL, 0);
        }
        if (ret == 0)
        {
            ret = spim_run(&crc8, 1, NULL, 0);
        }
    }

    cs_deassert();
    k_sem_give(&spi_lock);

    return ret;
}

int32_t dw3000_spi_write(uint16_t headerLength, const uint8_t* headerBuffer, uint16_t bodyLength, const uint8_t* bodyBuffer)
{
    const size_t total = (size_t)headerLength + bodyLength;
    int ret;

    k_sem_take(&spi_lock, K_FOREVER);
    cs_assert();

    if (total <= SPI_SCRATCH_LEN)
    {
        memcpy(tx_scratch, headerBuffer, headerLength);
        if (bodyLength)
        {
            memcpy(&tx_scratch[headerLength], bodyBuffer, bodyLength);
        }

        ret = spim_run(tx_scratch, total, NULL, 0);
    }
    else
    {
        ret = spim_run(headerBuffer, headerLength, NULL, 0);
        if (ret == 0)
        {
            ret = spim_run(bodyBuffer, bodyLength, NULL, 0);
        }
    }

    cs_deassert();
    k_sem_give(&spi_lock);

    return ret;
}

int32_t dw3000_spi_read(uint16_t headerLength, uint8_t* headerBuffer, uint16_t readLength, uint8_t* readBuffer)
{
    const size_t total = (size_t)headerLength + readLength;
    int ret;

    k_sem_take(&spi_lock, K_FOREVER);
    cs_assert();

    if (total <= SPI_SCRATCH_LEN)
    {
        /* One transaction: clock the header out and the whole frame in, then
         * drop the bytes received while the header was going out. */
        ret = spim_run(headerBuffer, headerLength, rx_scratch, total);
        if (ret == 0)
        {
            memcpy(readBuffer, &rx_scratch[headerLength], readLength);
        }
    }
    else
    {
        ret = spim_run(headerBuffer, headerLength, NULL, 0);
        if (ret == 0)
        {
            ret = spim_run(NULL, 0, readBuffer, readLength);
        }
    }

    cs_deassert();
    k_sem_give(&spi_lock);

    return ret;
}

void dw3000_spi_wakeup()
{
    /* CS pin should be configured as active low
     * To wake up, we set CS to 1, which will pull it low, for 500us */
#if KERNEL_VERSION_MAJOR > 3 || (KERNEL_VERSION_MAJOR == 3 && KERNEL_VERSION_MINOR >= 4)
    gpio_pin_set_dt(&cs_ctrl.gpio, 1);
    k_sleep(K_USEC(500));
    gpio_pin_set_dt(&cs_ctrl.gpio, 0);
#else
    gpio_pin_set_dt(&cs_ctrl->gpio, 1);
    k_sleep(K_USEC(500));
    gpio_pin_set_dt(&cs_ctrl->gpio, 0);
#endif
}
