#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(rjtag, LOG_LEVEL_INF);

#include <stm32_ll_dma.h>
#include <stm32_ll_spi.h>
#include <stm32_ll_tim.h>

#include "platform_stm32.h"

#ifdef RJTAG_STM32_USE_TIMER_SPI

#include "platform_impl.h"

#pragma message("Using STM32 specific timer + spi")
#pragma message("TODO: Double buffer this")

extern "C"
{
void OnPulsesCompleteIsr(void *arg); // Forward declare for setup
}

namespace
{
namespace Hardware
{

// This configures how many ticks it takes for a full SCK/TCK clock period
// This is after the prescaler set in the timer's overlay

#pragma message("TODO: Fix bug where FullClockPeriod_Ticks = 2 does not work")
constexpr unsigned FullClockPeriod_Ticks = 4;
constexpr unsigned HalfClockPeriod_Ticks = FullClockPeriod_Ticks / 2;

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

    // STM32 PWM driver starts the counter to be free running on init, so we need
    // to disable it
    LL_TIM_DisableCounter(TimerAddress);
    LL_TIM_SetUpdateSource(TimerAddress, LL_TIM_UPDATESOURCE_COUNTER);
    // Reset back to 0
    LL_TIM_SetCounter(TimerAddress, 0);
    LL_TIM_GenerateEvent_UPDATE(TimerAddress);
    LL_TIM_ClearFlag_UPDATE(TimerAddress);

    // Connect the interrupt so that Zephyr knows about it
    IRQ_CONNECT(IMPL_TIMER_UPDATE_IRQ, IMPL_TIMER_UPDATE_PRIORITY, OnPulsesCompleteIsr, nullptr, 0);
    irq_enable(IMPL_TIMER_UPDATE_IRQ);
    // Set up the interrupt for it in the hardware
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
    State::BitsToOnlySCK = (8 - (clockPulses & 7)) & 7;

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

struct DmaChannelDetails
{
    uint32_t Channel;
    uint32_t Slot;
    uint32_t ChannelConfig;

    constexpr DmaChannelDetails(uint32_t channel, uint32_t slot, uint32_t channelConfig)
        : Channel{channel}, Slot{slot}, ChannelConfig{channelConfig}
    {
    }
};

enum class SpiDirection
{
    TransmitOnly,
    Bidirectional,
};

struct SpiDetails
{
    uint32_t SpiBaseAddress;
    uint32_t DmaBaseAddress;
    SpiDirection Direction;
    DmaChannelDetails DmaRx;
    DmaChannelDetails DmaTx;

    constexpr SpiDetails(uint32_t spiBaseAddress, uint32_t dmaBaseAddress, SpiDirection direction,
                         DmaChannelDetails dmaRx, DmaChannelDetails dmaTx)
        : SpiBaseAddress{spiBaseAddress}, DmaBaseAddress{dmaBaseAddress}, Direction{direction}, DmaRx{dmaRx},
          DmaTx{dmaTx}
    {
    }

    auto SpiAddress() const
    {
        return reinterpret_cast<SPI_TypeDef *>(SpiBaseAddress);
    }

    auto DmaAddress() const
    {
        return reinterpret_cast<DMA_TypeDef *>(DmaBaseAddress);
    }
};

#define DMA_CHANNEL_DETAILS_FROM_DT(node, name)                                                                        \
    DmaChannelDetails                                                                                                  \
    {                                                                                                                  \
        DT_DMAS_CELL_BY_NAME(node, name, channel), DT_DMAS_CELL_BY_NAME(node, name, slot),                             \
            DT_DMAS_CELL_BY_NAME(node, name, channel_config)                                                           \
    }

#define SPI_DETAILS_FROM_DT(node, direction)                                                                           \
    SpiDetails                                                                                                         \
    {                                                                                                                  \
        DT_REG_ADDR(node), DT_REG_ADDR(DT_DMAS_CTLR(node)), direction, DMA_CHANNEL_DETAILS_FROM_DT(node, rx),          \
            DMA_CHANNEL_DETAILS_FROM_DT(node, tx)                                                                      \
    }

constexpr auto SpiTmsDetails = SPI_DETAILS_FROM_DT(DT_ALIAS(rjtagtms), SpiDirection::TransmitOnly);
constexpr auto SpiTdioDetails = SPI_DETAILS_FROM_DT(DT_ALIAS(rjtagtdio), SpiDirection::Bidirectional);

template <SpiDetails spiDetail> static inline void InitialiseSpiDevice()
{
    auto spi = spiDetail.SpiAddress();

    // Disable everything first
    spi->CR1 = 0;
    // No interrupts used, SPI in "motorola mode" (as opposed to "TI" mode), and DMA events
    // generated on tx empty/rx not empty
    spi->CR2 = SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;
}

template <uint32_t dmaBaseAddress, DmaChannelDetails dmaChannel> inline DMA_Stream_TypeDef *GetDmaStream()
{
    static_assert((dmaBaseAddress == DMA1_BASE) || (dmaBaseAddress == DMA2_BASE), "Unknown DMA base address");
    static_assert(dmaChannel.Channel < 8, "Only 8 dma channels");

    if constexpr (dmaBaseAddress == DMA1_BASE)
    {
        constexpr DMA_Stream_TypeDef *streamAddresses[] = {
            DMA1_Stream0, DMA1_Stream1, DMA1_Stream2, DMA1_Stream3,
            DMA1_Stream4, DMA1_Stream5, DMA1_Stream6, DMA1_Stream7,
        };

        return streamAddresses[dmaChannel.Channel];
    }

    if constexpr (dmaBaseAddress == DMA2_BASE)
    {
        constexpr DMA_Stream_TypeDef *streamAddresses[] = {
            DMA2_Stream0, DMA2_Stream1, DMA2_Stream2, DMA2_Stream3,
            DMA2_Stream4, DMA2_Stream5, DMA2_Stream6, DMA2_Stream7,
        };

        return streamAddresses[dmaChannel.Channel];
    }

    return nullptr;
}

template <uint32_t dmaBaseAddress, DmaChannelDetails dmaChannel> inline void ClearDmaFlags()
{
    constexpr bool highRegister = (dmaChannel.Channel >= 4);
    constexpr int setIndex = highRegister ? (dmaChannel.Channel - 4) : dmaChannel.Channel;
    constexpr int bitShift = (setIndex == 0) ? 0 : (setIndex == 1) ? 6 : (setIndex == 2) ? 16 : 22;

    constexpr uint32_t clearValue = 0b111101 << bitShift;

    auto dma = reinterpret_cast<DMA_TypeDef *>(dmaBaseAddress);

    if (highRegister)
    {
        dma->HIFCR = clearValue;
    }
    else
    {
        dma->LIFCR = clearValue;
    }
}

template <uint32_t dmaBaseAddress, DmaChannelDetails dmaChannel> static inline void DisableDmaChannel()
{
    auto stream = GetDmaStream<dmaBaseAddress, dmaChannel>();
    stream->CR = 0;
    ClearDmaFlags<dmaBaseAddress, dmaChannel>();
}

template <SpiDetails spiDetail> static inline void DisableDmaChannels()
{
    DisableDmaChannel<spiDetail.DmaBaseAddress, spiDetail.DmaTx>();

    if constexpr (spiDetail.Direction == SpiDirection::Bidirectional)
    {
        DisableDmaChannel<spiDetail.DmaBaseAddress, spiDetail.DmaRx>();
    }
}

template <SpiDetails spiDetail, DmaChannelDetails dmaChannel, typename DataBuffer>
static inline void SetupDmaChannel(unsigned totalBytes, DataBuffer buffer)
{
    constexpr uint32_t dataTransferDirection = dmaChannel.ChannelConfig & DMA_SxCR_DIR_Msk;
    // ??? Not a bug - checking for the value of MEMORY_TO_MEMORY because Zephyr seems to map
    // STM32_DMA_PERIPH_TO_MEMORY to the value of memory to memory?
    static_assert(
        (dataTransferDirection == LL_DMA_DIRECTION_MEMORY_TO_MEMORY) ||
            (dataTransferDirection == LL_DMA_DIRECTION_MEMORY_TO_PERIPH),
        "DMA channel configuration word must have a direction of memory -> peripheral, or peripheral -> memory");

    // So in the reference manual (rm0383), the "peripheral to memory" value is 0, but STM32_DMA_PERIPH_TO_MEMORY
    // defines it as 2.  Remap that.  Luckily, "memory to peripheral" stays at 1 in both mappings.

    constexpr uint32_t remappedChannelConfig = (dataTransferDirection == LL_DMA_DIRECTION_MEMORY_TO_MEMORY)
                                                   ? (dmaChannel.ChannelConfig & ~DMA_SxCR_DIR_Msk)
                                                   : dmaChannel.ChannelConfig;
    // DMA is the flow controller.  No need for interrupts, because the timer that generates the SCK pulses and
    // deasserts NSS knows when the SPI is finished.
    constexpr uint32_t channelConfig = remappedChannelConfig | (dmaChannel.Slot << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_EN;

    auto stream = GetDmaStream<spiDetail.DmaBaseAddress, dmaChannel>();

    stream->NDTR = totalBytes;
    stream->PAR = spiDetail.SpiBaseAddress + offsetof(SPI_TypeDef, DR);
    stream->M0AR = reinterpret_cast<uint32_t>(buffer);
    stream->CR = channelConfig;
}

template <SpiDetails spiDetail>
static inline void EnableSpiDevice(unsigned totalBytes, const void *misoBuffer, void *mosiBuffer)
{
    auto spi = spiDetail.SpiAddress();

    // Temporarily disable everything while we set up
    spi->CR1 = 0;
    DisableDmaChannels<spiDetail>();

    SetupDmaChannel<spiDetail, spiDetail.DmaTx>(totalBytes, misoBuffer);
    if constexpr (spiDetail.Direction == SpiDirection::Bidirectional)
    {
        __ASSERT(mosiBuffer != nullptr, "Bidirectional SPI, but NULL mosi buffer");
        SetupDmaChannel<spiDetail, spiDetail.DmaRx>(totalBytes, mosiBuffer);
    }

    // Set up for CPOL = 0, CPHA = 1, LSB first, 8-bit mode, no CRC
    constexpr uint32_t cr1 = SPI_CR1_SPE | SPI_CR1_LSBFIRST | SPI_CR1_CPHA;
    // When transmit-only, BIDIMODE = 0 and RXONLY = 0 (i.e. ignore MOSI pin)
    // as per reference manual.  No need to apply special control flags here.

    spi->CR1 = cr1;
}

void InitialiseSPI()
{
    // Use Zephyr's SPI drivers to initialise the system - i.e. turned on the
    // clocks for the SPI devices, and configured the GPIO pins.
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

    InitialiseSpiDevice<SpiTmsDetails>();
    InitialiseSpiDevice<SpiTdioDetails>();
}

void SetupJtagSpi(unsigned totalBits, const uint8_t *tms, const uint8_t *tdi, uint8_t *tdo)
{
    const unsigned bytesToSend = (totalBits + 7) >> 3;

    EnableSpiDevice<SpiTmsDetails>(bytesToSend, tms, nullptr);
    EnableSpiDevice<SpiTdioDetails>(bytesToSend, tdi, tdo);
}

} // namespace Hardware
} // namespace

extern "C"
{
void OnPulsesCompleteIsr(void *arg)
{
    LL_TIM_ClearFlag_UPDATE(Hardware::TimerAddress);

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
}

bool PlatformImpl_HasShiftPacket()
{
    // Still have to enable both DMA clocks manually
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;

#pragma message("Verify that SPI NSS/SCK pins are connected")
    // TODO: Sanity check by making sure that NSS pins are connected
    // TODO: Sanity check to make sure that pins are connected together at startup
    //       Do this by having the timer have 24 SCK bits, and checking the DMA NDTR
    //       register (need to send 3 bytes, because the SPI peripheral effectively
    //       buffers 2 bytes for Tx)

    Hardware::InitialiseTimer();
    // Timer needs to be initialised before SPI, as the communications clock needs
    // to be set to the idle polarity before SPI is enabled, as per reference manual.
    Hardware::InitialiseSPI();

    return true;
}

namespace
{

#pragma message("TODO: Double buffer this")
constexpr unsigned MaxBytesInTapShiftBuffer = 64;

struct SpiBuffers
{
    uint8_t Tms[MaxBytesInTapShiftBuffer];
    uint8_t Tdi[MaxBytesInTapShiftBuffer];
    uint8_t Tdo[MaxBytesInTapShiftBuffer];
};

SpiBuffers SpiBuffer;

} // namespace

int PlatformImpl_TapShiftPacket(void *privateData, const uint8_t *buffer, int bitsToShift)
{
    constexpr int MaxBitsInBuffer = MaxBytesInTapShiftBuffer * 8;

    for (; bitsToShift > 0; bitsToShift -= MaxBitsInBuffer)
    {
        const int bitsToParse = (bitsToShift > MaxBitsInBuffer) ? MaxBitsInBuffer : bitsToShift;
        const int bytesToParse = (bitsToParse + 7) >> 3;

        for (int bytesParsed = 0; bytesParsed < bytesToParse; ++bytesParsed, buffer += 2)
        {
            SpiBuffer.Tdi[bytesParsed] = buffer[0];
            SpiBuffer.Tms[bytesParsed] = buffer[1];
        }

        Hardware::SetupJtagSpi(bitsToParse, SpiBuffer.Tms, SpiBuffer.Tdi, SpiBuffer.Tdo);
        Hardware::StartClockingBits(bitsToParse);

        k_sem_take(&Hardware::State::Lock, K_FOREVER);

#pragma message("TODO: Verify that dma to tdo has completed by checking NDTR")

        k_sem_give(&Hardware::State::Lock);

        PlatformImpl_TransmitData(privateData, SpiBuffer.Tdo, bytesToParse);
    }

    return 0;
}
}

#endif