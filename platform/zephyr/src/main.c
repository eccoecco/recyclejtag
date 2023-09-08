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

#include "platform_impl.h"
#include "serial_queue.h"

#include <rjcore/rjcore.h>

LOG_MODULE_REGISTER(rjtag, LOG_LEVEL_INF);

struct serial_queue usb_rx; //!< Received over USB (to us)
struct serial_queue usb_tx; //!< Send over USB (to host)

static void on_uart_interrupt(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);

    while (uart_irq_update(dev) && uart_irq_is_pending(dev))
    {
        if (uart_irq_rx_ready(dev))
        {
            uint8_t buffer[64];
            int length;

            length = uart_fifo_read(dev, buffer, sizeof(buffer));
            if (length < 0)
            {
                LOG_ERR("Error reading USB UART: %d", length);
            }
            else if (length > 0)
            {
                int result = serial_queue_write(&usb_rx, buffer, length);

                if (result != length)
                {
                    LOG_ERR("Failed to write to internal serial queue - got %d, expected %d", result, length);
                }

                // LOG_INF("Rx %d [%02x %02x %02x %02x %02x %02x]", length, buffer[0], buffer[1], buffer[2], buffer[3],
                //         buffer[4], buffer[5]);
            }
        }

        if (uart_irq_tx_ready(dev))
        {
            uint8_t buffer[64];
            int length;

            length = serial_queue_read(&usb_tx, buffer, sizeof(buffer), K_NO_WAIT);

            if (length > 0)
            {
                int result = uart_fifo_fill(dev, buffer, length);
                if (result != length)
                {
                    LOG_ERR("Failed to write to serial port - got %d, expected %d", result, length);
                }
                // LOG_INF("Tx %d [%02x %02x %02x %02x %02x %02x]", length, buffer[0], buffer[1], buffer[2], buffer[3],
                //         buffer[4], buffer[5]);
            }
            else
            {
                uart_irq_tx_disable(dev);
            }
        }
    }
}

void PlatformImpl_TransmitData(void *privateData, const void *buffer, size_t length)
{
    const struct device *dev;
    dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);

    serial_queue_write(&usb_tx, buffer, length);
    uart_irq_tx_enable(dev);
}

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#define SW0_NODE DT_ALIAS(sw0)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec sw = GPIO_DT_SPEC_GET(SW0_NODE, gpios);

static struct RJCoreHandle rjcoreHandle;

int main(void)
{
    const struct device *dev;
    struct RJCorePlatform *rjcorePlatform;
    int ret;

    serial_queue_init(&usb_rx);
    serial_queue_init(&usb_tx);

    gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&sw, GPIO_INPUT);

    gpio_pin_set_dt(&led, 0);

    rjcorePlatform = PlatformImpl_Init();

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

    RJCore_Init(&rjcoreHandle, rjcorePlatform, NULL);

    while (true)
    {
        uint8_t buffer[64];

        int bytesRead = serial_queue_read(&usb_rx, buffer, sizeof(buffer), K_MSEC(100));

        RJCore_NotifyDataReceived(&rjcoreHandle, buffer, bytesRead);

        gpio_pin_toggle_dt(&led);
    }

    return 0;
}

#if 0
void testDMA(void)
{
    // So this is a bit tricky, because I'm going to use direct hardware register
    // access, and this may conflict with Zephyr drivers.  This is also not as
    // portable between the different STM32 platforms, which is a bit of a pain.

    // Need to use TIM1, because that's the only source of timers that can trigger
    // DMA2.  We need to use DMA2, because DMA1 does not have access to the GPIO
    // registers.

    TIM_TypeDef *timerBase = TIM1;

    const uint16_t cr1 = timerBase->CR1;

    if ((cr1 & TIM_CR1_CEN) != 0)
    {
        LOG_ERR("Something else is already using the timer!");
        return;
    }

    LOG_INF("Timer free for DMA/GPIO");

    {
        uint32_t currentApb = RCC->APB2ENR;
        LOG_INF("Current APB: 0x%08x", currentApb);
        currentApb |= RCC_APB2ENR_TIM1EN;
        RCC->APB2ENR = currentApb;
        LOG_INF("    New APB: 0x%08x", currentApb);
    }
    {
        // Must use DMA2, because DMA1 is not connected to the GPIO bases
        uint32_t currentAhb = RCC->AHB1ENR;
        LOG_INF("Current AHB: 0x%08x", currentAhb);
        currentAhb |= RCC_AHB1ENR_DMA2EN;
        RCC->AHB1ENR = currentAhb;
        LOG_INF("    New AHB: 0x%08x", currentAhb);

        LOG_INF("  Stream 0x%08x", DMA2_Stream0->CR);
        LOG_INF("  Stream 0x%08x", DMA2_Stream1->CR);
        LOG_INF("  Stream 0x%08x", DMA2_Stream2->CR);
        LOG_INF("  Stream 0x%08x", DMA2_Stream3->CR);
        LOG_INF("  Stream 0x%08x", DMA2_Stream4->CR);
        LOG_INF("  Stream 0x%08x", DMA2_Stream5->CR);
        LOG_INF("  Stream 0x%08x", DMA2_Stream6->CR);
        LOG_INF("  Stream 0x%08x", DMA2_Stream7->CR);
    }

    {
        // TIM1_UP is DMA 2, stream 5, channel 6

        // TODO: Verify stream 5 unused
        DMA2_Stream5->CR = 0;

        //        static const uint32_t clearValue = (0x10000 << led.pin);
        // LED is active low, so setting the pin turns the LED off
        static const uint32_t clearValue = (0x1 << led.pin);

        DMA2_Stream5->NDTR = 4; // Can be used to make sure that the GPIO is cleared
        DMA2_Stream5->PAR = (uint32_t)(&(GPIOC->BSRR));
        DMA2_Stream5->M0AR = (uint32_t)(&clearValue);
        DMA2_Stream5->FCR = 0; // Direct mode

        DMA2->HIFCR = 0x0F400;

#if 0
        DMA2_Stream5->CR = (6 << DMA_SxCR_CHSEL_Pos) | (2 << DMA_SxCR_MSIZE_Pos) | (2 << DMA_SxCR_PSIZE_Pos) |
                           DMA_SxCR_CIRC | (1 << DMA_SxCR_DIR_Pos) | DMA_SxCR_EN;
#else
        DMA2_Stream5->CR = (6 << DMA_SxCR_CHSEL_Pos) | (2 << DMA_SxCR_MSIZE_Pos) | (2 << DMA_SxCR_PSIZE_Pos) |
                           (1 << DMA_SxCR_DIR_Pos) | DMA_SxCR_EN;
#endif
        LOG_INF("Reconf stream 5: 0x%08x (from 0x%08x to 0x%08x)", DMA2_Stream5->CR, DMA2_Stream5->M0AR,
                DMA2_Stream5->PAR);
    }
    {
        // TIM1_CH1 is DMA 2, stream 1/3 channel 6, or stream 6, channel 0
        DMA2_Stream1->CR = 0;

        static const uint32_t setValue = (0x10000 << led.pin);

        DMA2_Stream1->NDTR = 4;
        DMA2_Stream1->PAR = (uint32_t)(&(GPIOC->BSRR));
        DMA2_Stream1->M0AR = (uint32_t)(&setValue);
        DMA2_Stream1->FCR = 0; // Direct mode

        DMA2->LIFCR = 0x0F400;

        DMA2_Stream1->CR = (6 << DMA_SxCR_CHSEL_Pos) | (2 << DMA_SxCR_MSIZE_Pos) | (2 << DMA_SxCR_PSIZE_Pos) |
                           (1 << DMA_SxCR_DIR_Pos) | DMA_SxCR_EN;
        LOG_INF("Reconf stream 1: 0x%08x (from 0x%08x to 0x%08x)", DMA2_Stream1->CR, DMA2_Stream1->M0AR,
                DMA2_Stream1->PAR);
    }

    static uint32_t gpioAIDR = 0;
    {
        // TIM1_CH2 is DMA 2, stream 2 channel 6, or stream 6, channel 0 (shared with TIM1_CH{1,2,3})
        DMA2_Stream2->CR = 0;

        DMA2_Stream2->NDTR = 4;
        DMA2_Stream2->PAR = (uint32_t)(&(GPIOA->IDR));
        DMA2_Stream2->M0AR = (uint32_t)(&gpioAIDR);
        DMA2_Stream2->FCR = 0; // Direct mode

        DMA2->LIFCR = 0x0F400;

        DMA2_Stream2->CR = (6 << DMA_SxCR_CHSEL_Pos) | (2 << DMA_SxCR_MSIZE_Pos) | (2 << DMA_SxCR_PSIZE_Pos) |
                           DMA_SxCR_CIRC | (0 << DMA_SxCR_DIR_Pos) | DMA_SxCR_EN;
        LOG_INF("Reconf stream 2: 0x%08x (from 0x%08x to 0x%08x)", DMA2_Stream2->CR, DMA2_Stream2->M0AR,
                DMA2_Stream2->PAR);
    }

    // Set the timer to be pretty much default up-counting with its clock from the system PLL
    timerBase->CR1 = TIM_CR1_URS; // Set this so that updating registers via EGR will not send an update event
    timerBase->CR2 = 0;
    timerBase->SMCR = 0;

    timerBase->DIER = TIM_DIER_UDE | TIM_DIER_CC1DE | TIM_DIER_CC2DE; // TODO: Set up DMA requests for UDE, CC1DE, CC2DE

    timerBase->PSC = 47999; // 96MHz system clock / 48kHz = 2000 ticks to get a 1s period
    timerBase->ARR = 1999;  // 1s period
    timerBase->CNT = 0;

    timerBase->CCER = 0;   // No output compare
    timerBase->CCR1 = 999; // 50% of the way through
    timerBase->CCR2 = 1900;

    timerBase->EGR =
        TIM_EGR_UG; // Update registers now, otherwise update events will fire to the DMA immediately upon enabling
    LOG_INF("Status: 0x%08x", timerBase->SR);

    timerBase->SR = 0; // Clear all status flags

    timerBase->CR1 = TIM_CR1_CEN | TIM_CR1_URS; // Enable timer (TODO: Set URS)
    LOG_INF("Start: %d %d", DMA2_Stream5->NDTR, DMA2_Stream1->NDTR);

    while (true)
    {
        const uint32_t sr = timerBase->SR;

        if ((sr & TIM_SR_UIF) != 0)
        {
            timerBase->SR = 0;

            LOG_INF("Overflow 0x%08x: %d %d %d", sr, DMA2_Stream5->NDTR, DMA2_Stream1->NDTR, DMA2_Stream2->NDTR);

            // Key is active low, so invert
            LOG_INF("User button: %d", (gpioAIDR & 1) == 0);
        }

        if ((sr & TIM_SR_CC1IF) != 0)
        {
            timerBase->SR = 0;
            LOG_INF("CC1IF 0x%08x: %d %d %d", sr, DMA2_Stream5->NDTR, DMA2_Stream1->NDTR, DMA2_Stream2->NDTR);
        }

        k_msleep(1);
    }

    timerBase->CR1 = 0;
}

#endif