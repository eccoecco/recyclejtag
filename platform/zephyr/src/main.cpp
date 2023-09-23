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
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/uart/cdc_acm.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>

#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>

#include "platform_impl.h"
#include "serial_queue.h"

#include <stm32_ll_tim.h>

#include <rjcore/rjcore.h>

LOG_MODULE_REGISTER(rjtag, LOG_LEVEL_INF);

#if 0

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
#endif

extern "C"
{
void OnPulsesCompleteIsr(void *arg); // Forward declare for setup
}

namespace Hardware
{
namespace Pwms
{
#define SHIM_PWM_DT_SPEC_GET_BY_IDX(node, _, idx) PWM_DT_SPEC_GET_BY_IDX(node, idx)

const pwm_dt_spec Sck[] = {DT_FOREACH_PROP_ELEM_SEP(DT_NODELABEL(rjtagsck), pwms, SHIM_PWM_DT_SPEC_GET_BY_IDX, (, ))};
const pwm_dt_spec Tck[] = {DT_FOREACH_PROP_ELEM_SEP(DT_NODELABEL(rjtagtck), pwms, SHIM_PWM_DT_SPEC_GET_BY_IDX, (, ))};

} // namespace Pwms

constexpr uint32_t TimerBaseAddress = DT_REG_ADDR(DT_ALIAS(rjtagtimer));
const auto TimerAddress = reinterpret_cast<TIM_TypeDef *>(TimerBaseAddress);

enum class PwmTarget
{
    Tck, //!< This timer is for generating TCK
    Sck, //!< This timer is for generating SCK
};

inline void ForEachPwm(auto callback)
{
    // Iterated through this order so that dependent timers set up their
    // trigger inputs before the trigger outputs of their parent timers
    // are configured
    for (const auto &pwm : Pwms::Sck)
    {
        callback(pwm, PwmTarget::Sck);
    }
    for (const auto &pwm : Pwms::Tck)
    {
        callback(pwm, PwmTarget::Tck);
    }
}

constexpr unsigned FullClockPeriod_Ticks = 2000;
constexpr unsigned HalfClockPeriod_Ticks = FullClockPeriod_Ticks / 2;

void ConfigurePWM(const pwm_dt_spec &pwmDeviceTree, Hardware::PwmTarget pwmTarget)
{
    (void)pwmTarget;
    pwm_set_cycles(pwmDeviceTree.dev, pwmDeviceTree.channel, FullClockPeriod_Ticks, HalfClockPeriod_Ticks,
                   pwmDeviceTree.flags);
}

#define IMPL_TIMER_UPDATE_IRQ DT_IRQ_BY_NAME(DT_ALIAS(rjtagtimer), up, irq)
#define IMPL_TIMER_UPDATE_PRIORITY DT_IRQ_BY_NAME(DT_ALIAS(rjtagtimer), up, priority)

void InitialiseTimer()
{
    // STM32 PWM driver starts the counter to be free running on init
    LL_TIM_DisableCounter(TimerAddress);
    LL_TIM_SetUpdateSource(TimerAddress, LL_TIM_UPDATESOURCE_COUNTER);
    // Reset b ack to 0
    LL_TIM_SetCounter(TimerAddress, 0);
    LL_TIM_GenerateEvent_UPDATE(TimerAddress);
    LL_TIM_ClearFlag_UPDATE(TimerAddress);

    // Connect the interrupt so that Zephyr knows about it
    IRQ_CONNECT(IMPL_TIMER_UPDATE_IRQ, IMPL_TIMER_UPDATE_PRIORITY, OnPulsesCompleteIsr, nullptr, 0);
    irq_enable(IMPL_TIMER_UPDATE_IRQ);
    // Set up the hardware for it
    LL_TIM_EnableIT_UPDATE(TimerAddress);

    Hardware::ForEachPwm(ConfigurePWM);
}

static void UpdateChannel(int channel, unsigned value)
{
    switch (channel)
    {
    case 1:
        TimerAddress->CCR1 = value;
        break;
    case 2:
        TimerAddress->CCR2 = value;
        break;
    case 3:
        TimerAddress->CCR3 = value;
        break;
    case 4:
        TimerAddress->CCR4 = value;
        break;
    default:
        __ASSERT(false, "Unknown timer channel %d", channel);
        break;
    }
}

static void SetupTimer(unsigned clockPulses, bool bothOutputs)
{
    __ASSERT(clockPulses <= 256, "Clock pulses must be an 8-bit number, but got %d", clockPulses);
    __ASSERT(clockPulses > 0, "Clock pulses cannot be 0");

    for (const auto &tckPwm : Pwms::Tck)
    {
        if (bothOutputs)
        {
            // Set 50% duty cycle
            UpdateChannel(tckPwm.channel, HalfClockPeriod_Ticks - 1);
        }
        else
        {
            // Keep idle high
            UpdateChannel(tckPwm.channel, FullClockPeriod_Ticks);
        }
    }

    LL_TIM_SetRepetitionCounter(TimerAddress, clockPulses - 1);
    LL_TIM_SetOnePulseMode(TimerAddress, LL_TIM_ONEPULSEMODE_SINGLE);
    LL_TIM_GenerateEvent_UPDATE(TimerAddress);
    LL_TIM_EnableCounter(TimerAddress);
}

} // namespace Hardware

extern "C"
{
void OnPulsesCompleteIsr(void *arg)
{
    LL_TIM_ClearFlag_UPDATE(Hardware::TimerAddress);

    LOG_INF("isr");
}
}

int main(void)
{
    LOG_INF("Initialising...");
    // usb_enable(NULL);

    Hardware::InitialiseTimer();

    Hardware::SetupTimer(4, true);

#if 0
    LOG_INF("Timer 1 dump");
    LOG_INF("CR1:   %08x", Hardware::TimerAddress->CR1);
    LOG_INF("CR2:   %08x", Hardware::TimerAddress->CR2);
    LOG_INF("SMCR:  %08x", Hardware::TimerAddress->SMCR);
    LOG_INF("DIER:  %08x", Hardware::TimerAddress->DIER);
    LOG_INF("CCMR1: %08x", Hardware::TimerAddress->CCMR1);
    LOG_INF("CCMR2: %08x", Hardware::TimerAddress->CCMR2);
    LOG_INF("CCER:  %08x", Hardware::TimerAddress->CCER);
    LOG_INF("CCR1:  %08x", Hardware::TimerAddress->CCR1);
    LOG_INF("CCR2:  %08x", Hardware::TimerAddress->CCR2);
    LOG_INF("CCR3:  %08x", Hardware::TimerAddress->CCR3);
    LOG_INF("CCR4:  %08x", Hardware::TimerAddress->CCR4);
    LOG_INF("PSC:   %08x", Hardware::TimerAddress->PSC);
    LOG_INF("ARR:   %08x", Hardware::TimerAddress->ARR);
#endif

    LOG_INF("TCK SCK");

    for (int i = 0; i < 100; ++i)
    {
        const uint32_t idr = GPIOB->IDR;
        bool tck = idr & (1 << 13);
        bool sck0 = idr & (1 << 0);
        bool sck1 = idr & (1 << 1);

        LOG_INF("%d   %d %d", tck, sck0, sck1);

        k_msleep(100);
    }

    while (true)
    {
        k_sleep(K_SECONDS(100));
    }

    return 0;
}