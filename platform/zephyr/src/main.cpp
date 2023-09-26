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
#include <zephyr/drivers/spi.h>
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
namespace Gpios
{
struct Details
{
    GPIO_TypeDef *PortAddress;
    gpio_dt_spec DtSpec;
};

#define GET_GPIO_DETAILS_BY_IDX(node_id, prop, idx)                                                                    \
    Details                                                                                                            \
    {                                                                                                                  \
        .PortAddress = reinterpret_cast<GPIO_TypeDef *>(DT_REG_ADDR(DT_GPIO_CTLR_BY_IDX(node_id, prop, idx))),         \
        .DtSpec = GPIO_DT_SPEC_GET_BY_IDX(node_id, prop, idx)                                                          \
    }

const Details Nss[] = {DT_FOREACH_PROP_ELEM_SEP(DT_NODELABEL(rjtagnss), gpios, GET_GPIO_DETAILS_BY_IDX, (, ))};
} // namespace Gpios
namespace State
{

// Locked with the IRQ is running.  The IRQ will unlock it when done.
// Use a semaphore since k_sem_give() is isr-ok
k_sem Lock;

// Should only be written to if lock is acquired (free access from IRQ because
// IRQ implicitly acquires lock)
unsigned BitsToBothClocks = 0;
unsigned BitsToOnlySCK = 0;
unsigned TotalBitsClocked = 0;

static constexpr unsigned MaximumBitsPerRepetition = 256;

} // namespace State

constexpr uint32_t TimerBaseAddress = DT_REG_ADDR(DT_ALIAS(rjtagtimer));
const auto TimerAddress = reinterpret_cast<TIM_TypeDef *>(TimerBaseAddress);

constexpr uint32_t SpiTmsBaseAddress = DT_REG_ADDR(DT_ALIAS(rjtagtms));
const auto SpiTmsAddress = reinterpret_cast<SPI_TypeDef *>(SpiTmsBaseAddress);
constexpr uint32_t SpiTmsDmaController = DT_REG_ADDR(DT_DMAS_CTLR(DT_ALIAS(rjtagtms)));

constexpr uint32_t SpiTdioBaseAddress = DT_REG_ADDR(DT_ALIAS(rjtagtdio));
const auto SpiTdioAddress = reinterpret_cast<SPI_TypeDef *>(SpiTdioBaseAddress);
constexpr uint32_t SpiTdioDmaController = DT_REG_ADDR(DT_DMAS_CTLR(DT_ALIAS(rjtagtdio)));
constexpr uint32_t SpiTdioDmaRxChannel = DT_DMAS_CELL_BY_NAME(DT_ALIAS(rjtagtdio), rx, channel);
constexpr uint32_t SpiTdioDmaRxSlot = DT_DMAS_CELL_BY_NAME(DT_ALIAS(rjtagtdio), rx, slot);
constexpr uint32_t SpiTdioDmaRxChannelConfig = DT_DMAS_CELL_BY_NAME(DT_ALIAS(rjtagtdio), rx, channel_config);
// There is also a "features" field, but that's for FIFO thresholds.  Let's not use the FIFO unless we have to.
// TODO: Macros for grabbing all this information
// TODO: tx channel as well

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

constexpr unsigned FullClockPeriod_Ticks = 2;
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
    for (const auto &nssSpec : Gpios::Nss)
    {
        gpio_pin_configure_dt(&nssSpec.DtSpec, GPIO_OUTPUT_INACTIVE);
    }

    k_sem_init(&State::Lock, 1, 1);
    State::TotalBitsClocked = 0;
    State::BitsToBothClocks = 0;
    State::BitsToOnlySCK = 0;

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

static void SetupBitsToClock()
{
    unsigned clockPulses = State::BitsToBothClocks;

    if (clockPulses > 0)
    {
        if (clockPulses > State::MaximumBitsPerRepetition)
        {
            // Can only send out 256 pulses at a time, as the repetition counter
            // is 8 bits only
            clockPulses = State::MaximumBitsPerRepetition;
        }
        State::BitsToBothClocks -= clockPulses;

        LOG_INF("Setup %d bits to both clocks", clockPulses);
    }
    else
    {
        clockPulses = State::BitsToOnlySCK;
        State::BitsToOnlySCK = 0;

        LOG_INF("Setup %d bits to SCK only", clockPulses);
    }

    LL_TIM_SetRepetitionCounter(TimerAddress, clockPulses - 1);
    LL_TIM_SetOnePulseMode(TimerAddress, LL_TIM_ONEPULSEMODE_SINGLE);
    LL_TIM_GenerateEvent_UPDATE(TimerAddress);
    LL_TIM_EnableCounter(TimerAddress);
}

static void StartClockingBits(unsigned clockPulses)
{
    __ASSERT(clockPulses > 0, "Clock pulses cannot be 0");

    k_sem_take(&State::Lock, K_FOREVER);

    State::TotalBitsClocked = clockPulses;
    State::BitsToBothClocks = clockPulses;
    State::BitsToOnlySCK = 8 - (clockPulses & 7);

    for (const auto &nssSpec : Gpios::Nss)
    {
        // Active low, so clear to be active
        nssSpec.PortAddress->BSRR = (0x1'0000 << nssSpec.DtSpec.pin);
        // gpio_pin_set_dt(&nssSpec, 1);
    }

    for (const auto &tckPwm : Pwms::Tck)
    {
        // Set 50% duty cycle
        UpdateChannel(tckPwm.channel, HalfClockPeriod_Ticks - 1);
    }

    SetupBitsToClock();
}

void InitialiseSPI()
{
    // Ensure that Zephyr has turned on the clocks for the SPI devices, and
    // configured the GPIO pins.
    //   At this time, even though SPI has an async/poll mechanism, the STM32
    // driver for spi_transceive_signal() is interrupt based, and does *NOT*
    // use DMA!  The problem is, at medium-ish SPI clock speeds (~1MHz), ISR
    // based SPI transceive is not fast enough to reliably keep up.
    //   If I want DMA based SPI transfer, I either use the blocking
    // spi_transceive(), which would require me to spawn 2 separate threads
    // (one for each blocking SPI operation), or I roll my own driver... and
    // so I roll my own userspace driver.
    const device *tmsDevice = DEVICE_DT_GET(DT_ALIAS(rjtagtms));
    if (!device_is_ready(tmsDevice))
    {
        LOG_ERR("SPI for TMS is not ready");
        return;
    }

    const device *tdioDevice = DEVICE_DT_GET(DT_ALIAS(rjtagtdio));
    if (!device_is_ready(tdioDevice))
    {
        LOG_ERR("SPI for TDI/TDO is not ready");
        return;
    }
}

void WaitUntilSpiDone()
{
    if (!Spi::TransactionInProgress)
    {
        return;
    }

#if 0
    while (true)
    {
        k_sem_take(&State::Lock, K_FOREVER);

        if ((Hardware::State::BitsToBothClocks == 0) && (Hardware::State::BitsToOnlySCK == 0))
        {
            break;
        }

        if (Hardware::State::BitsToBothClocks == 0)
        {
            // Only clock pulses to sck left
            // Set TCK to idle high when doing leftover bits
            for (const auto &tckPwm : Hardware::Pwms::Tck)
            {
                Hardware::UpdateChannel(tckPwm.channel, Hardware::FullClockPeriod_Ticks);
            }
        }

        Hardware::SetupBitsToClock();
    }

    for (const auto &nssSpec : Hardware::Gpios::Nss)
    {
        // Is this ISR safe in general?
        // In the case of the STM32, I think it is, but it may not be guaranteed
        // in all platforms.
        // gpio_pin_set_dt(&nssSpec, 0);

        // Direct register access should be safe
        nssSpec.PortAddress->BSRR = (0x1 << nssSpec.DtSpec.pin);
    }

    k_sem_give(&State::Lock);
#endif

    k_poll_event event;
    k_poll_event_init(&event, K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &Spi::SignalDone);

    k_poll(&event, 1, K_FOREVER);

    k_poll_signal_reset(&Spi::SignalDone);

    Spi::TransactionInProgress = false;
}

void SetupJtagSpi(unsigned totalBits, const uint8_t *tms, const uint8_t *tdi, uint8_t *tdo, uint8_t *discarded)
{
    const device *tmsDevice = DEVICE_DT_GET(DT_ALIAS(rjtagtms));
    const device *tdioDevice = DEVICE_DT_GET(DT_ALIAS(rjtagtdio));

    unsigned bytesToSend = (totalBits + 7) >> 3;

    const spi_buf tmsBuffer = {
        .buf = const_cast<uint8_t *>(tms),
        .len = bytesToSend,
    };
    const spi_buf_set tmsBuffers = {
        .buffers = &tmsBuffer,
        .count = 1,
    };

    const spi_buf tdiBuffer = {
        .buf = const_cast<uint8_t *>(tdi),
        .len = bytesToSend,
    };
    const spi_buf_set tdiBuffers = {
        .buffers = &tdiBuffer,
        .count = 1,
    };

    spi_buf tdoBuffer = {
        .buf = tdo,
        .len = bytesToSend,
    };
    const spi_buf_set tdoBuffers = {
        .buffers = &tdoBuffer,
        .count = 1,
    };
    spi_buf discardedBuffer = {
        .buf = discarded,
        .len = bytesToSend,
    };
    const spi_buf_set discardedBuffers = {
        .buffers = &discardedBuffer,
        .count = 1,
    };

    WaitUntilSpiDone();

    int result;

    result = spi_transceive_signal(tmsDevice, &Spi::Config, &tmsBuffers, &discardedBuffers, nullptr);

    __ASSERT(result == 0, "Spi tms transceive failure: %d", result);

    result = spi_transceive_signal(tdioDevice, &Spi::Config, &tdiBuffers, &tdoBuffers, &Spi::SignalDone);

    __ASSERT(result == 0, "Spi tdi/tdo transceive failure: %d", result);

    if (result == 0)
    {
        Spi::TransactionInProgress = true;
    }
}

} // namespace Hardware

extern "C"
{
void OnPulsesCompleteIsr(void *arg)
{
    LL_TIM_ClearFlag_UPDATE(Hardware::TimerAddress);
    LOG_INF("isr");

#if 1
    if ((Hardware::State::BitsToBothClocks == 0) && (Hardware::State::BitsToOnlySCK == 0))
    {
        for (const auto &nssSpec : Hardware::Gpios::Nss)
        {
            // Is this ISR safe in general?
            // In the case of the STM32, I think it is, but it may not be guaranteed
            // in all platforms.
            // gpio_pin_set_dt(&nssSpec, 0);

            // Direct register access should be safe
            nssSpec.PortAddress->BSRR = (0x1 << nssSpec.DtSpec.pin);
        }

        k_sem_give(&Hardware::State::Lock);
    }
    else
    {
        if (Hardware::State::BitsToBothClocks == 0)
        {
            // Only clock pulses to sck left
            // Set TCK to idle high when doing leftover bits
            for (const auto &tckPwm : Hardware::Pwms::Tck)
            {
                Hardware::UpdateChannel(tckPwm.channel, Hardware::FullClockPeriod_Ticks);
            }
        }

        Hardware::SetupBitsToClock();
    }
#else
    k_sem_give(&Hardware::State::Lock);
#endif
}
}

constexpr unsigned testSize = 5;
const uint8_t tmsBuffer[testSize] = {0x50, 0x05, 0xaa, 0x55, 0x22};
const uint8_t tdiBuffer[testSize] = {0x33, 0x22, 0x11, 0x22, 0x33};
uint8_t tdoBuffer[testSize] = {0xee, 0xee, 0xee, 0xee, 0xee};
uint8_t discardedBuffer[testSize]; // Async transceive for STM32 seems to need a destination to write to

int main(void)
{
    // usb_enable(NULL);

    LOG_INF("RCC CFGR: %08x", RCC->CFGR);

    LOG_INF("OSPEEDR: %08x", GPIOB->OSPEEDR);
    LOG_INF("PUPDR:   %08x", GPIOB->PUPDR);

    Hardware::InitialiseTimer();
    // Timer needs to be initialised before SPI, as the communications clock needs
    // to be set to the idle polarity before SPI is enabled, as per reference manual.
    Hardware::InitialiseSPI();

    const unsigned bitsToTest = testSize * 8 - 3;

    Hardware::SetupJtagSpi(bitsToTest, tmsBuffer, tdiBuffer, tdoBuffer, discardedBuffer);
    Hardware::StartClockingBits(bitsToTest);

#if 0
    LOG_INF("NSS1,2 SCK TCK TMS TDI TDO");

    for (int i = 0; i < 130; ++i)
    {
        const uint32_t idra = GPIOA->IDR;
        const uint32_t idrb = GPIOB->IDR;
        bool tck = idrb & (1 << 13);
        bool tms = idrb & (1 << 4);
        bool tdi = idrb & (1 << 14);
        bool tdo = idrb & (1 << 15);
        bool sck0 = idrb & (1 << 0);
        bool sck1 = idrb & (1 << 1);
        bool nss1 = idra & (1 << 3);
        bool nss2 = idrb & (1 << 8);

        LOG_INF("   %d,%d %d,%d  %d   %d   %d   %d", nss1, nss2, sck0, sck1, tck, tms, tdi, tdo);

        k_msleep(100);

        {
            unsigned signalled;
            int result;

            k_poll_signal_check(&Hardware::Spi::SignalDone, &signalled, &result);

            if (signalled != 0)
            {
                k_poll_signal_reset(&Hardware::Spi::SignalDone);
                LOG_INF("SPI Done!");
                for (unsigned i = 0; i < testSize; ++i)
                {
                    LOG_INF("%d: %02x", i, tdoBuffer[i]);
                }
            }
        }
    }
#endif

    Hardware::WaitUntilSpiDone();
    LOG_INF("SPI Done!");
    for (unsigned i = 0; i < testSize; ++i)
    {
        LOG_INF("%d: %02x (expected %02x)", i, tdoBuffer[i], tdiBuffer[i]);
    }

    while (true)
    {
        k_sleep(K_SECONDS(100));
    }

    return 0;
}

// TODO: Make sure that DMA is actually configured for SPI
//   - Not for async - looks like it's only for transceive()?!
// TODO: Move SPI/Timer stuff into own file
// TODO: Sanity check to make sure that pins are connected together at startup