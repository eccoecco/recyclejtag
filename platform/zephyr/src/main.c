/*
    Simple GPIO based bit bashing for RJTag, based on USB CDC
*/
/*
 * Was originally USB CDC ACM sample:
 *
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/uart/cdc_acm.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>

#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>

LOG_MODULE_REGISTER(rjtag, LOG_LEVEL_INF);

static void on_uart_interrupt(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);

    while (uart_irq_update(dev) && uart_irq_is_pending(dev))
    {
        if (uart_irq_rx_ready(dev))
        {
            int recv_len, rb_len;
            uint8_t buffer[64];

            recv_len = uart_fifo_read(dev, buffer, sizeof(buffer));
            if (recv_len < 0)
            {
                LOG_ERR("Failed to read UART FIFO");
                recv_len = 0;
            };
        }

        if (uart_irq_tx_ready(dev))
        {
#if 0
            uint8_t buffer[64];
            int rb_len, send_len;

            rb_len = ring_buf_get(&tx_ringbuf, buffer, sizeof(buffer));
            if (!rb_len)
            {
                LOG_DBG("Ring buffer empty, disable TX IRQ");
                uart_irq_tx_disable(dev);
                continue;
            }

            send_len = uart_fifo_fill(dev, buffer, rb_len);
            if (send_len < rb_len)
            {
                LOG_ERR("Drop %d bytes", rb_len - send_len);
            }

            LOG_DBG("ringbuf -> tty fifo %d bytes", send_len);
#endif
        }
    }
}

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

int main(void)
{
    const struct device *dev;
    int ret;

    gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);

    gpio_pin_set_dt(&led, 0);

    dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
    if (!device_is_ready(dev))
    {
        LOG_ERR("CDC ACM device not ready");
        return 0;
    }

    ret = usb_enable(NULL);

    if (ret != 0)
    {
        LOG_ERR("Failed to enable USB");
        return 0;
    }

    uart_irq_callback_set(dev, on_uart_interrupt);

    /* Enable rx interrupts */
    uart_irq_rx_enable(dev);

    while (true)
    {
        k_sleep(K_SECONDS(10));
    }

    return 0;
}