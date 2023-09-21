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

// #define CLOCKS_FULL_SPEED
// Want a 1MHz clock, but do some shenanigans with phases, so want 4 ticks
// The system clock is 96MHz, so 96/4 = 24, so the prescaler is 24-1=23
#ifdef CLOCKS_FULL_SPEED
static const unsigned TimerPrescaler = 23;
static const unsigned TimerPeriod = 3;
#else
// Debugging at 1Hz
static const unsigned TimerPrescaler = 47999;
static const unsigned TimerPeriod = 1999;
#endif

// Do it via direct register access right now, as Zephyr's timer API doesn't expose trigger
// functionality.
// I was going to say that Zephyr doesn't support SPI slave functionality, but it does seem
// like it's been added since I last looked?
//   We can probably use Zephyr to configure and power up the timers, and then we get in there
// with direct register access and apply our modifications to enable triggers.  That way, we
// claim the resources the Zephyr way, and only write the custom code that hooks up the triggers
// and enables things the way we need.

static void EnableTimersOnAPB()
{
    const uint32_t apb1Mask =
        RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN | RCC_APB1ENR_TIM5EN | RCC_APB1ENR_SPI2EN;
    const uint32_t apb2Mask = RCC_APB2ENR_TIM1EN | RCC_APB2ENR_SPI1EN;

    uint32_t apb1 = RCC->APB1ENR;
    LOG_INF("Initial APB1ENR: 0x%08x", apb1);
    if ((apb1 & apb1Mask) != 0)
    {
        LOG_WRN("One of timers 2-5 or spi 2 seem to already be in use by Zephyr?");
    }
    apb1 |= apb1Mask;
    // RCC->APB1ENR = apb1;

    uint32_t apb2 = RCC->APB2ENR;
    LOG_INF("Initial APB2ENR: 0x%08x", apb2);
    if ((apb2 & apb2Mask) != 0)
    {
        LOG_WRN("Timer 1 or spi 1 already seems to be in use by Zephyr?");
    }
    apb2 |= apb2Mask;
    // RCC->APB2ENR = apb2;

    // LOG_INF("Timers 1-5, spi 1-2 peripherals enabled");
}

static void ConfigurePulseTimer(TIM_TypeDef *timerBase, unsigned inputTrigger, unsigned outputTriggerChannel,
                                bool enableBothChannels)
{
    const unsigned CCMR_OCM_PWMMode2 = 7;
    const unsigned TimerHalfPeriod = (TimerPeriod + 1) / 2 - 1;

    timerBase->CR1 = 0; // Disable timer
    timerBase->CR2 = outputTriggerChannel << TIM_CR2_MMS_Pos;
    timerBase->SMCR = 0; // Some bits in SMCR say they should only be changed when SMS == 0
    timerBase->SMCR =
        (inputTrigger
         << TIM_SMCR_TS_Pos); // 5 or 6 << TIM_SMCR_SMS_Pos; // TODO: Change this for gated or trigger mode as needs be
    timerBase->DIER = 0;
    timerBase->CCMR1 = CCMR_OCM_PWMMode2 << TIM_CCMR1_OC1M_Pos; // Set PWM mode 2
    timerBase->CCMR2 = enableBothChannels ? (CCMR_OCM_PWMMode2 << TIM_CCMR2_OC4M_Pos) : 0;
    timerBase->CCER = TIM_CCER_CC1E | (enableBothChannels ? TIM_CCER_CC4E : 0);

    timerBase->CNT = 0;
    timerBase->PSC = TimerPrescaler;
    timerBase->ARR = TimerPeriod;

    timerBase->CCR1 = TimerHalfPeriod;
    timerBase->CCR4 = TimerHalfPeriod;

    timerBase->SR = 0;
    timerBase->EGR = 0;
}

namespace Hardware
{

struct PwmDetails
{
    uint32_t TimerAddress;
    pwm_dt_spec DtSpec;
};

namespace Pwms
{

#define PWM_DETAILS_GET_BY_IDX(node, _, idx)                                                                           \
    PwmDetails                                                                                                         \
    {                                                                                                                  \
        .TimerAddress = DT_REG_ADDR(DT_PARENT(DT_PWMS_CTLR_BY_IDX(node, idx))),                                        \
        .DtSpec = PWM_DT_SPEC_GET_BY_IDX(node, idx),                                                                   \
    }
// The STM32 PWM controller is a child of the timer, hence we need to go up to the parent to get the proper address

const PwmDetails Sck[] = {DT_FOREACH_PROP_ELEM_SEP(DT_NODELABEL(rjtagsck), pwms, PWM_DETAILS_GET_BY_IDX, (, ))};
const PwmDetails SckCounter[] = {
    DT_FOREACH_PROP_ELEM_SEP(DT_NODELABEL(rjtagsckcounter), pwms, PWM_DETAILS_GET_BY_IDX, (, ))};
const PwmDetails Tck[] = {DT_FOREACH_PROP_ELEM_SEP(DT_NODELABEL(rjtagtck), pwms, PWM_DETAILS_GET_BY_IDX, (, ))};
const PwmDetails TckCounter[] = {
    DT_FOREACH_PROP_ELEM_SEP(DT_NODELABEL(rjtagtckcounter), pwms, PWM_DETAILS_GET_BY_IDX, (, ))};
const PwmDetails Source[] = {DT_FOREACH_PROP_ELEM_SEP(DT_NODELABEL(rjtagsource), pwms, PWM_DETAILS_GET_BY_IDX, (, ))};

} // namespace Pwms

// Timer addresses in order
const uint32_t TimerAddresses[] = {DT_REG_ADDR(DT_NODELABEL(timers1)), DT_REG_ADDR(DT_NODELABEL(timers2)),
                                   DT_REG_ADDR(DT_NODELABEL(timers3)), DT_REG_ADDR(DT_NODELABEL(timers4)),
                                   DT_REG_ADDR(DT_NODELABEL(timers5))};

static constexpr int Timer1Offset = 0;
static constexpr int Timer2Offset = 1;
static constexpr int Timer3Offset = 2;
static constexpr int Timer4Offset = 3;
static constexpr int Timer5Offset = 4;

// Timer offset is off by 1 compared to what the timer is called.
// i.e. timer1 = 0 offset, timer2 = 1 offset, etc
inline int FindTimerOffsetByAddress(TIM_TypeDef *timerAddress)
{
    const auto numericAddress = reinterpret_cast<uint32_t>(timerAddress);

    int index = 0;

    for (auto address : TimerAddresses)
    {
        if (address == numericAddress)
        {
            return index;
        }
        ++index;
    }

    return -1;
}

// Given a destination timer offset and which timer is meant to be its source, returns
// either a valid ITR mask or UINT32_MAX if no connection can be established to it
inline uint32_t FindItrByTimerOffset(int destinationTimerOffset, int sourceTimerOffset)
{
    constexpr auto MaximumTimerOffset = ARRAY_SIZE(TimerAddresses);

    __ASSERT((destinationTimerOffset >= 0) && (destinationTimerOffset < MaximumTimerOffset),
             "Invalid destination timer offset");
    __ASSERT((sourceTimerOffset >= 0) && (sourceTimerOffset < MaximumTimerOffset), "Invalid source timer offset");

    switch (destinationTimerOffset)
    {
    case Timer1Offset:
        switch (sourceTimerOffset)
        {
        case Timer5Offset:
            return LL_TIM_TS_ITR0;
        case Timer2Offset:
            return LL_TIM_TS_ITR1;
        case Timer3Offset:
            return LL_TIM_TS_ITR2;
        case Timer4Offset:
            return LL_TIM_TS_ITR3;
        }
        return UINT32_MAX;
    case Timer2Offset:
        switch (sourceTimerOffset)
        {
        case Timer1Offset:
            return LL_TIM_TS_ITR0;
        case Timer3Offset:
            return LL_TIM_TS_ITR2;
        case Timer4Offset:
            return LL_TIM_TS_ITR3;
        }
        return UINT32_MAX;
    case Timer3Offset:
        switch (sourceTimerOffset)
        {
        case Timer1Offset:
            return LL_TIM_TS_ITR0;
        case Timer2Offset:
            return LL_TIM_TS_ITR1;
        case Timer5Offset:
            return LL_TIM_TS_ITR2;
        case Timer4Offset:
            return LL_TIM_TS_ITR3;
        }
        return UINT32_MAX;
    case Timer4Offset:
        switch (sourceTimerOffset)
        {
        case Timer1Offset:
            return LL_TIM_TS_ITR0;
        case Timer2Offset:
            return LL_TIM_TS_ITR1;
        case Timer3Offset:
            return LL_TIM_TS_ITR2;
        }
        return UINT32_MAX;
    case Timer5Offset:
        switch (sourceTimerOffset)
        {
        case Timer2Offset:
            return LL_TIM_TS_ITR0;
        case Timer3Offset:
            return LL_TIM_TS_ITR1;
        case Timer4Offset:
            return LL_TIM_TS_ITR2;
        }
        return UINT32_MAX;
    }

    return UINT32_MAX;
}

enum class PwmRole
{
    OutputPin, //!< Used to drive output PWMs
    Counter,   //!< Used to count how many pulses have been generated
    Source,    //!< The source sync of the entire timer tree
};

enum class PwmTarget
{
    Tck,  //!< This timer is for generating TCK
    Sck,  //!< This timer is for generating SCK
    Both, //!< Only applicable to Source
};

inline void ForEachPwm(auto callback)
{
    // Iterated through this order so that dependent timers set up their
    // trigger inputs before the trigger outputs of their parent timers
    // are configured
    for (const auto &pwm : Pwms::Sck)
    {
        callback(pwm, PwmTarget::Sck, PwmRole::OutputPin);
    }
    for (const auto &pwm : Pwms::SckCounter)
    {
        callback(pwm, PwmTarget::Sck, PwmRole::Counter);
    }
    for (const auto &pwm : Pwms::Tck)
    {
        callback(pwm, PwmTarget::Tck, PwmRole::OutputPin);
    }
    for (const auto &pwm : Pwms::TckCounter)
    {
        callback(pwm, PwmTarget::Tck, PwmRole::Counter);
    }
    for (const auto &pwm : Pwms::Source)
    {
        callback(pwm, PwmTarget::Both, PwmRole::Source);
    }
}

void ConfigurePWM(const Hardware::PwmDetails &pwmDetails, Hardware::PwmTarget pwmTarget, Hardware::PwmRole pwmRole)
{
    __ASSERT(device_is_ready(pwmDetails.DtSpec.dev), "Timer device not ready");

    constexpr uint32_t ExternalClockMode1 = TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0;
    auto timerAddress = reinterpret_cast<TIM_TypeDef *>(pwmDetails.TimerAddress);

    auto selectInputTrigger = [](TIM_TypeDef *destination, Hardware::PwmTarget pwmTarget, Hardware::PwmRole pwmRole) {
        if ((destination->SMCR & TIM_SMCR_SMS) != 0)
        {
            // Already set up - ignore
            return;
        }

        const auto sourceAddress =
            (pwmRole == Hardware::PwmRole::Counter) ? Hardware::Pwms::Source->TimerAddress
            : ((pwmTarget == Hardware::PwmTarget::Sck) && (pwmRole == Hardware::PwmRole::OutputPin))
                ? Hardware::Pwms::SckCounter->TimerAddress
            : ((pwmTarget == Hardware::PwmTarget::Tck) && (pwmRole == Hardware::PwmRole::OutputPin))
                ? Hardware::Pwms::TckCounter->TimerAddress
                : 0;

        __ASSERT(sourceAddress != 0, "Unknown role/target to assign input trigger");
        auto source = reinterpret_cast<TIM_TypeDef *>(sourceAddress);

        const int destinationTimerOffset = Hardware::FindTimerOffsetByAddress(destination);
        const int sourceTimerOffset = Hardware::FindTimerOffsetByAddress(source);

        __ASSERT(destinationTimerOffset != -1, "Invalid destination timer address 0x%08x", destination);
        __ASSERT(sourceTimerOffset != -1, "Invalid source timer address 0x%08x", source);

        const uint32_t itr = Hardware::FindItrByTimerOffset(destinationTimerOffset, sourceTimerOffset);

        __ASSERT(itr != UINT32_MAX, "Unknown connection from TIMER%d to TIMER%d", sourceTimerOffset + 1,
                 destinationTimerOffset + 1);

        if (itr != UINT32_MAX)
        {
            LOG_INF("Connecting TIMER%d --> TRGO --> ITR%d --> TIMER%d", sourceTimerOffset + 1, itr >> TIM_SMCR_TS_Pos,
                    destinationTimerOffset + 1);
            LL_TIM_SetTriggerInput(destination, itr);
        }
    };

    switch (pwmRole)
    {
    case Hardware::PwmRole::OutputPin:
        selectInputTrigger(timerAddress, pwmTarget, pwmRole);
        LL_TIM_SetSlaveMode(timerAddress, LL_TIM_SLAVEMODE_GATED);
        break;
    case Hardware::PwmRole::Counter:
        selectInputTrigger(timerAddress, pwmTarget, pwmRole);
        if ((timerAddress->CR2 & TIM_CR2_MMS_Msk) == 0)
        {
            __ASSERT((pwmDetails.DtSpec.channel >= 1) && (pwmDetails.DtSpec.channel <= 4),
                     "Pwm channel must be between 1 and 4, but was %d", pwmDetails.DtSpec.channel);

            const uint32_t trgo = (pwmDetails.DtSpec.channel == 1)   ? LL_TIM_TRGO_OC1REF
                                  : (pwmDetails.DtSpec.channel == 2) ? LL_TIM_TRGO_OC2REF
                                  : (pwmDetails.DtSpec.channel == 3) ? LL_TIM_TRGO_OC3REF
                                  : (pwmDetails.DtSpec.channel == 4) ? LL_TIM_TRGO_OC4REF
                                                                     : 0;

            LL_TIM_SetTriggerOutput(timerAddress, trgo);
        }
        // All counters get their clocks from the source clock
        LL_TIM_SetSlaveMode(timerAddress, ExternalClockMode1);
        // One pulse mode (will have to set every call)
        LL_TIM_SetOnePulseMode(timerAddress, LL_TIM_ONEPULSEMODE_SINGLE);

        break;
    case Hardware::PwmRole::Source:
        LL_TIM_SetTriggerOutput(timerAddress, LL_TIM_TRGO_UPDATE);
        break;
    }
}

} // namespace Hardware

int main(void)
{
    LOG_INF("Initialising...");
    // usb_enable(NULL);

    Hardware::ForEachPwm(Hardware::ConfigurePWM);

    /*
    ```
              +-> TIMER5 --> TIMER3 -> SCK
              |   ITR0       ITR2
    TIMER2 ---+
              |
              +-> TIMER1 --> TIMER4 -> TCK
                  ITR1       ITR0
    ```
    */

#if 0
    LL_TIM_SetTriggerInput(TIM3, LL_TIM_TS_ITR2);
    LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_GATED);
    LL_TIM_SetTriggerInput(TIM4, LL_TIM_TS_ITR0);
    LL_TIM_SetSlaveMode(TIM4, LL_TIM_SLAVEMODE_GATED);

    const uint32_t ExternalClockMode1 = TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0;

    LL_TIM_SetTriggerInput(TIM1, LL_TIM_TS_ITR1);
    LL_TIM_SetSlaveMode(TIM1, ExternalClockMode1);
    LL_TIM_SetTriggerInput(TIM5, LL_TIM_TS_ITR0);
    LL_TIM_SetSlaveMode(TIM5, ExternalClockMode1);

    LL_TIM_SetTriggerOutput(TIM5, LL_TIM_TRGO_OC1REF);
    LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_OC1REF);
    LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_UPDATE);

    LOG_INF("SMCR: %08x %08x", TIM3->SMCR, TIM4->SMCR);
    LOG_INF(" CR2: %08x %08x", TIM3->CR2, TIM4->CR2);

    LL_TIM_SetOnePulseMode(TIM1, LL_TIM_ONEPULSEMODE_SINGLE);
    LL_TIM_SetOnePulseMode(TIM5, LL_TIM_ONEPULSEMODE_SINGLE);

    for (int i = 0; i < 5; ++i)
    {
        // TODO: Swap to pwm_set_cycles to use clock cycles, not time
        int result = pwm_set_dt(&jtaghw_pwms[i], 1000000000, 500000000);

        k_msleep(100);

        LOG_INF("PWM%d: Result %d", i, result);
    }

    LOG_INF("SMCR: %08x %08x", TIM3->SMCR, TIM4->SMCR);
    LOG_INF(" CR2: %08x %08x", TIM3->CR2, TIM4->CR2);

    /*
        ConfigurePulseTimer(TIM3, 0, 0, true);
        ConfigurePulseTimer(TIM4, 0, 0, false);

        TIM3->CR1 = TIM_CR1_CEN;
        TIM4->CR1 = TIM_CR1_CEN;
    */
#endif

    while (true)
    {
        k_msleep(100);

        LOG_INF("PWM Pins on Port A: %08x B: %08x", GPIOA->IDR & (1 << 6), GPIOB->IDR & ((1 << 6) | (1 << 1)));
    }

    return 0;
}