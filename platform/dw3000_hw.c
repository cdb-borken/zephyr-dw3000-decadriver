#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <core_cm4.h>

#include "deca_device_api.h"
#include "dw3000_hw.h"
#include "dw3000_spi.h"

LOG_MODULE_REGISTER(dw3000, CONFIG_DW3000_LOG_LEVEL);

#define DW_INST DT_INST(0, decawave_dw3000)

static struct gpio_callback gpio_cb;
static struct k_work dw3000_isr_work;

/* The ISR must not share the system workqueue: BLE, USB/RNDIS and networking
 * work items queued ahead of it delay TX/RX completion by an unbounded amount. */
#define DW3000_ISR_WORKQ_STACK_SIZE 2048
#define DW3000_ISR_WORKQ_PRIORITY K_PRIO_COOP(4)
#define DW3000_ISR_MAX_PASSES 8

static K_THREAD_STACK_DEFINE(dw3000_isr_workq_stack, DW3000_ISR_WORKQ_STACK_SIZE);
static struct k_work_q dw3000_isr_workq;

struct dw3000_config
{
    struct gpio_dt_spec gpio_irq;
    struct gpio_dt_spec gpio_reset;
    struct gpio_dt_spec gpio_wakeup;
    struct gpio_dt_spec gpio_spi_pol;
    struct gpio_dt_spec gpio_spi_pha;
};

static const struct dw3000_config conf = {
    .gpio_irq = GPIO_DT_SPEC_GET_OR(DW_INST, irq_gpios, {0}),
    .gpio_reset = GPIO_DT_SPEC_GET_OR(DW_INST, reset_gpios, {0}),
    .gpio_wakeup = GPIO_DT_SPEC_GET_OR(DW_INST, wakeup_gpios, {0}),
    .gpio_spi_pol = GPIO_DT_SPEC_GET_OR(DW_INST, spi_pol_gpios, {0}),
    .gpio_spi_pha = GPIO_DT_SPEC_GET_OR(DW_INST, spi_pha_gpios, {0}),
};

int dw3000_hw_init(void)
{
    /* Reset */
    if (conf.gpio_reset.port)
    {
        gpio_pin_configure_dt(&conf.gpio_reset, GPIO_INPUT);
        LOG_INF("RESET on %s pin %d", conf.gpio_reset.port->name, conf.gpio_reset.pin);
    }

    /* Wakeup (optional) */
    if (conf.gpio_wakeup.port)
    {
        gpio_pin_configure_dt(&conf.gpio_wakeup, GPIO_OUTPUT_ACTIVE);
        LOG_INF("WAKEUP on %s pin %d", conf.gpio_wakeup.port->name, conf.gpio_wakeup.pin);
    }

    /* SPI Polarity (optional) */
    if (conf.gpio_spi_pol.port)
    {
        gpio_pin_configure_dt(&conf.gpio_spi_pol, GPIO_OUTPUT_INACTIVE);
        LOG_INF("SPI_POL on %s pin %d", conf.gpio_spi_pol.port->name, conf.gpio_spi_pol.pin);
    }

    /* SPI Phase (optional) */
    if (conf.gpio_spi_pha.port)
    {
        gpio_pin_configure_dt(&conf.gpio_spi_pha, GPIO_OUTPUT_INACTIVE);
        LOG_INF("SPI_PHA on %s pin %d", conf.gpio_spi_pha.port->name, conf.gpio_spi_pha.pin);
    }

    return dw3000_spi_init();
}

static void dw3000_hw_isr_work_handler(struct k_work* item)
{
    /* Bounded: an enabled status bit that dwt_isr() does not clear keeps the IRQ
     * line asserted, and this queue is cooperative, so an unbounded loop would
     * starve every application thread. */
    int passes = 0;

    while (gpio_pin_get_dt(&conf.gpio_irq))
    {
        dwt_isr();

        if (++passes >= DW3000_ISR_MAX_PASSES)
        {
            LOG_WRN("IRQ still asserted after %d passes, unhandled event enabled?", passes);
            break;
        }
    }

    dw3000_isr_passes = passes;
    dw3000_isr_done_cycles = DWT->CYCCNT;
}

static void dw3000_hw_isr(const struct device* dev, struct gpio_callback* cb, uint32_t pins)
{
    dw3000_irq_cycles = DWT->CYCCNT;
    k_work_submit_to_queue(&dw3000_isr_workq, &dw3000_isr_work);
}

int dw3000_hw_init_interrupt(void)
{
    if (conf.gpio_irq.port)
    {
        k_work_queue_init(&dw3000_isr_workq);
        k_work_queue_start(&dw3000_isr_workq, dw3000_isr_workq_stack, K_THREAD_STACK_SIZEOF(dw3000_isr_workq_stack), DW3000_ISR_WORKQ_PRIORITY, NULL);
        k_thread_name_set(&dw3000_isr_workq.thread, "dw3000_isr");
        k_work_init(&dw3000_isr_work, dw3000_hw_isr_work_handler);

        gpio_pin_configure_dt(&conf.gpio_irq, GPIO_INPUT);
        gpio_init_callback(&gpio_cb, dw3000_hw_isr, BIT(conf.gpio_irq.pin));
        gpio_add_callback(conf.gpio_irq.port, &gpio_cb);
        gpio_pin_interrupt_configure_dt(&conf.gpio_irq, GPIO_INT_EDGE_RISING);

        LOG_INF("IRQ on %s pin %d", conf.gpio_irq.port->name, conf.gpio_irq.pin);
        return 0;
    }
    else
    {
        LOG_ERR("IRQ pin not configured");
        return -ENOENT;
    }
}

void dw3000_hw_interrupt_enable(void)
{
    if (conf.gpio_irq.port)
    {
        gpio_pin_interrupt_configure_dt(&conf.gpio_irq, GPIO_INT_EDGE_RISING);
    }
}

void dw3000_hw_interrupt_disable(void)
{
    if (conf.gpio_irq.port)
    {
        gpio_pin_interrupt_configure_dt(&conf.gpio_irq, GPIO_INT_DISABLE);
    }
}

bool dw3000_hw_interrupt_is_enabled(void)
{
    return true; // TODO
}

void dw3000_hw_fini(void)
{
    // TODO
    if (conf.gpio_irq.port)
    {
        gpio_pin_interrupt_configure_dt(&conf.gpio_irq, GPIO_INT_DISABLE);
        gpio_pin_configure_dt(&conf.gpio_irq, GPIO_DISCONNECTED);
    }
    if (conf.gpio_reset.port)
    {
        gpio_pin_configure_dt(&conf.gpio_reset, GPIO_DISCONNECTED);
    }
    if (conf.gpio_wakeup.port)
    {
        gpio_pin_configure_dt(&conf.gpio_wakeup, GPIO_DISCONNECTED);
    }

    dw3000_spi_fini();
}

void dw3000_hw_reset()
{
    if (!conf.gpio_reset.port)
    {
        LOG_ERR("No HW reset configured");
        return;
    }

    gpio_pin_configure_dt(&conf.gpio_reset, GPIO_OUTPUT_ACTIVE);
    k_msleep(1); // 10 us?
    gpio_pin_configure_dt(&conf.gpio_reset, GPIO_INPUT);
    k_msleep(2);
}

/** wakeup either using the WAKEUP pin or SPI CS */
void dw3000_hw_wakeup(void)
{
    if (conf.gpio_wakeup.port)
    {
        /* Use WAKEUP pin if available */
        LOG_INF("WAKEUP PIN");
        gpio_pin_set_dt(&conf.gpio_wakeup, 1);
        k_msleep(1);
        gpio_pin_set_dt(&conf.gpio_wakeup, 0);
    }
    else
    {
        /* Use SPI CS pin */
        LOG_INF("WAKEUP CS");
        dw3000_spi_wakeup();
    }
}

/** set WAKEUP pin low if available */
void dw3000_hw_wakeup_pin_low(void)
{
    if (conf.gpio_wakeup.port)
    {
        gpio_pin_set_dt(&conf.gpio_wakeup, 0);
    }
}
