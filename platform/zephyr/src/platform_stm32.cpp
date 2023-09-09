#include <zephyr/kernel.h>

#ifdef CONFIG_SOC_FAMILY_STM32

#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(rjtag, LOG_LEVEL_INF);

#include "platform_impl.h"

#include <cstddef>

#ifdef CONFIG_SOC_SERIES_STM32F4X
#define USE_DMA_GPIO_BITBASH 1
#endif

namespace
{

constexpr struct gpio_dt_spec rjTck = GPIO_DT_SPEC_GET(DT_NODELABEL(tck), gpios);
constexpr struct gpio_dt_spec rjTms = GPIO_DT_SPEC_GET(DT_NODELABEL(tms), gpios);
constexpr struct gpio_dt_spec rjTdo = GPIO_DT_SPEC_GET(DT_NODELABEL(tdo), gpios);
constexpr struct gpio_dt_spec rjTdi = GPIO_DT_SPEC_GET(DT_NODELABEL(tdi), gpios);

// For an MVP, let's just assume that all pins are on the same port.  There's no technical
// reason that they have to be, but it does just simplify the code a lot.
static_assert(rjTck.port == rjTms.port, "All jtag pins must be on the same port (TCK/TMS differs)");
static_assert(rjTck.port == rjTdo.port, "All jtag pins must be on the same port (TCK/TDO differs)");
static_assert(rjTck.port == rjTdi.port, "All jtag pins must be on the same port (TCK/TDI differs)");

// By assuming all pins are on the same port, we can just do this once:
constexpr auto rjPortAddress = DT_REG_ADDR(DT_GPIO_CTLR(DT_NODELABEL(tck), gpios));

// STM32s only define GPIOx_BASE if they're present on the device.  Use the BASE address
// before they're cast to GPIO_TypeDef * as we can directly compare ints with DT_REG_ADDR.
constexpr int validPortAddresses[] = {
#ifdef GPIOA_BASE
    GPIOA_BASE,
#endif
#ifdef GPIOB_BASE
    GPIOB_BASE,
#endif
#ifdef GPIOC_BASE
    GPIOC_BASE,
#endif
#ifdef GPIOD_BASE
    GPIOD_BASE,
#endif
#ifdef GPIOE_BASE
    GPIOE_BASE,
#endif
#ifdef GPIOF_BASE
    GPIOF_BASE,
#endif
#ifdef GPIOG_BASE
    GPIOG_BASE,
#endif
#ifdef GPIOH_BASE
    GPIOH_BASE,
#endif
#ifdef GPIOI_BASE
    GPIOI_BASE,
#endif
#ifdef GPIOJ_BASE
    GPIOJ_BASE,
#endif
#ifdef GPIOK_BASE
    GPIOK_BASE,
#endif
};

template <int testAddress> constexpr bool ensureValidPortAddress()
{
    for (auto portAddress : validPortAddresses)
    {
        if (testAddress == portAddress)
        {
            return true;
        }
    }

    return false;
}

// Just a sanity check to make sure that rjPortAddress points to a known GPIOx_BASE address
static_assert(ensureValidPortAddress<rjPortAddress>(), "Unable to find GPIO controller address in port addresses");

#ifdef USE_DMA_GPIO_BITBASH

enum class DMAStreamEventChannel
{
    Channel0 = 0,
    Channel1,
    Channel2,
    Channel3,
    Channel4,
    Channel5,
    Channel6,
    Channel7,
};

enum class DMAStreamDirection
{
    PeripheralToMemory,
    MemoryToPeripheral,
};

enum class DMACircularMode
{
    Disabled,
    Enabled,
};

enum class DMAMemoryIncrementMode
{
    Disabled,
    Enabled,
};

template <unsigned dmaStream> constexpr auto GetDMA2Stream()
{
    static_assert(dmaStream <= 7, "Only 7 DMA streams present");

    constexpr DMA_Stream_TypeDef *dmaStreams[] = {
        DMA2_Stream0, DMA2_Stream1, DMA2_Stream2, DMA2_Stream3, DMA2_Stream4, DMA2_Stream5, DMA2_Stream6, DMA2_Stream7,
    };

    return dmaStreams[dmaStream];
}

template <unsigned dmaStream> constexpr auto GetDMA2StreamFlagClearRegister()
{
    static_assert(dmaStream <= 7, "Only 7 DMA streams present");

    if constexpr (dmaStream < 4)
    {
        return &DMA2->LIFCR;
    }

    return &DMA2->HIFCR;
}

template <unsigned dmaStream> constexpr uint32_t GetDMA2StreamFlagClearBitmask()
{
    // This bitmask is common to all streams and shifted across the registers
    constexpr uint32_t clearBitmask = 0b111101;

    if constexpr ((dmaStream & 3) == 0)
    {
        return clearBitmask;
    }
    else if constexpr ((dmaStream & 3) == 1)
    {
        return clearBitmask << 6;
    }
    else if constexpr ((dmaStream & 3) == 2)
    {
        return clearBitmask << 16;
    }
    else
    {
        return clearBitmask << 22;
    }
}

// Configures the specified DMA stream.  Only a subset of the full DMA stream functionality is
template <unsigned streamNumber, DMAStreamEventChannel eventChannel, DMAStreamDirection direction,
          DMACircularMode circularMode, DMAMemoryIncrementMode memoryIncrement>
inline void ConfigureDMA2Stream(uint32_t peripheralAddress, void *memoryAddress, uint32_t wordsToTransfer)
{
    auto dmaStream = GetDMA2Stream<streamNumber>();
    auto fcr = GetDMA2StreamFlagClearRegister<streamNumber>();
    constexpr uint32_t flagClearBitmask = GetDMA2StreamFlagClearBitmask<streamNumber>();

    static_assert(sizeof(uintptr_t) == sizeof(uint32_t), "Memory address does not fit into 32-bit register?");

    const uint32_t dmaMemoryAddress = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(memoryAddress));

    // In our usage, it's always 32-bits memory/peripheral
    const uint32_t crValue = (static_cast<unsigned>(eventChannel) << DMA_SxCR_CHSEL_Pos) | (2 << DMA_SxCR_MSIZE_Pos) |
                             (2 << DMA_SxCR_PSIZE_Pos) | DMA_SxCR_EN | (3 << DMA_SxCR_PL_Pos) |
                             ((memoryIncrement == DMAMemoryIncrementMode::Enabled) ? DMA_SxCR_MINC : 0) |
                             ((direction == DMAStreamDirection::MemoryToPeripheral) ? (1 << DMA_SxCR_DIR_Pos) : 0) |
                             ((circularMode == DMACircularMode::Enabled) ? DMA_SxCR_CIRC : 0);

    LOG_DBG("Configuring stream %d @ %p : 0x%08x <-> 0x%08x", streamNumber, dmaStream, peripheralAddress,
            dmaMemoryAddress);
    LOG_DBG("  Flag clear register: %p = 0x%08x", fcr, flagClearBitmask);
    LOG_DBG("  Control register: 0x%08x", crValue);
    LOG_DBG("  Words: %d", wordsToTransfer);

    dmaStream->CR = 0;
    dmaStream->NDTR = wordsToTransfer;
    dmaStream->PAR = peripheralAddress;
    dmaStream->M0AR = dmaMemoryAddress;
    dmaStream->FCR = 0;

    *fcr = flagClearBitmask;

    dmaStream->CR = crValue;
}

namespace DMABuffers
{

uint32_t tckSet = (0x1 << rjTck.pin);

constexpr size_t MaximumBitsPerBuffer = 256;

static_assert((MaximumBitsPerBuffer & 7) == 0, "MaximumBitsPerBuffer must be divisible by 8");

enum class BufferMode
{
    Unused,
    Pending,
    InProgress,
};

struct TxBuffer
{
    uint32_t RawData[MaximumBitsPerBuffer];
    unsigned ValidWords = 0;
    BufferMode Mode = BufferMode::Unused;
};

TxBuffer TxBuffers[2];
int ActiveTxBufferIndex = 0;

uint32_t ReadData[MaximumBitsPerBuffer];
uint8_t ReassembledData[MaximumBitsPerBuffer / 8];

void SendData(const uint8_t *buffer, int bitsToShift)
{
    constexpr uint32_t tckClearMask = (0x1'0000 << rjTck.pin);
    constexpr uint32_t tdiSetMask = (1 << rjTdi.pin);
    constexpr uint32_t tdiClearMask = (0x1'0000 << rjTdi.pin);
    constexpr uint32_t tmsSetMask = (1 << rjTms.pin);
    constexpr uint32_t tmsClearMask = (0x1'0000 << rjTms.pin);

    __ASSERT(bitsToShift <= MaximumBitsPerBuffer, "Too many bits to shift!");

    // TODO: Proper double buffering
    TxBuffer &txBuffer = TxBuffers[0];

    txBuffer.ValidWords = bitsToShift;
    txBuffer.Mode = BufferMode::Pending;
    uint32_t *writeData = txBuffer.RawData;

    for (; bitsToShift > 0; bitsToShift -= 8, buffer += 2)
    {
        uint8_t tdi = buffer[0];
        uint8_t tms = buffer[1];

        for (int bitIndex = 0; bitIndex < 8; ++bitIndex, tdi >>= 1, tms >>= 1, ++writeData)
        {
            uint32_t bsrr = tckClearMask;
            bsrr |= ((tdi & 1) != 0) ? tdiSetMask : tdiClearMask;
            bsrr |= ((tms & 1) != 0) ? tmsSetMask : tmsClearMask;

            *writeData = bsrr;

            if (txBuffer.ValidWords < 16)
            {
                // LOG_INF("x: %08x", bsrr);
            }
        }
    }

    // DMA 2 Stream 1, Channel 6 = TIM1 CH1 (rising edge)
    ConfigureDMA2Stream<1, DMAStreamEventChannel::Channel6, DMAStreamDirection::MemoryToPeripheral,
                        DMACircularMode::Disabled, DMAMemoryIncrementMode::Enabled>(
        rjPortAddress + offsetof(GPIO_TypeDef, BSRR), txBuffer.RawData, txBuffer.ValidWords);
    // DMA 2 Stream 5, Channel 6 = TIM1 UP (falling edge pin sample)
    ConfigureDMA2Stream<5, DMAStreamEventChannel::Channel6, DMAStreamDirection::PeripheralToMemory,
                        DMACircularMode::Disabled, DMAMemoryIncrementMode::Enabled>(
        rjPortAddress + offsetof(GPIO_TypeDef, IDR), ReadData, txBuffer.ValidWords);

    auto timerBase = TIM1;

    timerBase->CNT = 0;
    timerBase->EGR = TIM_EGR_UG;
    timerBase->SR = 0;
    timerBase->CR1 = TIM_CR1_CEN | TIM_CR1_URS;

    while (DMA2_Stream5->NDTR != 0)
    {
        k_usleep(1);
        __NOP();
        // k_msleep(1000);
        // LOG_INF("Blarghle: %d %d %d", DMA2_Stream1->NDTR, DMA2_Stream2->NDTR, timerBase->CNT);
    }

    if constexpr (false)
    {
        const uint32_t idr = reinterpret_cast<GPIO_TypeDef *>(rjPortAddress)->IDR;

        LOG_INF("IDR: 0x%04x", idr);
    }

    // LOG_INF("Done");

    timerBase->CR1 = TIM_CR1_URS;

    for (int fillIndex = txBuffer.ValidWords; fillIndex < ((txBuffer.ValidWords + 7) & ~7); ++fillIndex)
    {
        ReadData[fillIndex] = 0;
    }

    const uint32_t *readBuffer = ReadData;
    uint8_t *finalBytes = ReassembledData;

    for (int bitsToProcess = txBuffer.ValidWords; bitsToProcess > 0; bitsToProcess -= 8, ++finalBytes)
    {
        constexpr uint32_t tdoInputMask = (1 << rjTdo.pin);
        constexpr uint32_t tdiInputMask = (1 << rjTdi.pin);
        constexpr uint32_t tmsInputMask = (1 << rjTms.pin);

        uint8_t tdo = 0;

        for (int bitIndex = 0; bitIndex < 8; ++bitIndex, ++readBuffer)
        {
            // if (txBuffer.ValidWords < 16)
            if constexpr (false)
            {
                // LOG_INF("%04x", static_cast<uint16_t>(*readBuffer));
                const uint32_t value = *readBuffer;
                LOG_INF("Tdi: %d - Tms: %d - Tdo: %d", (value & tdiInputMask) != 0, (value & tmsInputMask) != 0,
                        (value & tdoInputMask) != 0);
            }
            tdo >>= 1;
            tdo |= (((*readBuffer) & (1 << rjTdo.pin)) != 0) ? 0x80 : 0x00;
        }

        *finalBytes = tdo;

        // k_usleep(1);
    }

    // TODO: Pass privateData along
    PlatformImpl_TransmitData(nullptr, ReassembledData, (txBuffer.ValidWords + 7) / 8);
}

} // namespace DMABuffers

#endif

} // namespace

extern "C"
{

#ifdef USE_DMA_GPIO_BITBASH
// The 4xx series (I only have the F411 on hand, so this generalisation might not be true)
// can use DMA2 to write to the GPIO registers.  DMA2 must be triggered off TIM1, but because
// we're using them in ways that I can't quite describe to Zephyr right now, we check to see
// if some other driver is using these resources first, and back off if they are.

bool PlatformImpl_HasShiftPacket()
{
    uint32_t apb2 = RCC->APB2ENR;
    uint32_t ahb1 = RCC->AHB1ENR;

    if ((apb2 & RCC_APB2ENR_TIM1EN) != 0)
    {
        LOG_ERR("Timer 1 may already be in use!  Falling back to gpio mode");
        return false;
    }

    if ((ahb1 & RCC_AHB1ENR_DMA2EN) != 0)
    {
        // Technically, we can further check if the streams that we're interested
        // in are claimed, and only if there's collision, do we return false.
        // We can also go one step further, and determine which streams are free,
        // and then readjust the timer events generated to only use the free streams.
        // But this JTAG dongle tends to be single purpose, so ... I'm not going to
        // bother with that in this project.
        LOG_ERR("DMA2 may already be in use!  Falling back to gpio mode");
        return false;
    }

    // Technically, these checks are also non-exhaustive, because a timer can
    // be started after this check, and then the clock control may be turned on
    // to the peripheral then.  But this should be good enough for now.

    apb2 |= RCC_APB2ENR_TIM1EN;
    ahb1 |= RCC_AHB1ENR_DMA2EN;
    RCC->APB2ENR = apb2;
    RCC->AHB1ENR = ahb1;

    ConfigureDMA2Stream<2, DMAStreamEventChannel::Channel6, DMAStreamDirection::MemoryToPeripheral,
                        DMACircularMode::Enabled, DMAMemoryIncrementMode::Disabled>(
        rjPortAddress + offsetof(GPIO_TypeDef, BSRR), &DMABuffers::tckSet, 1);

    auto timerBase = TIM1;

    // Set the timer to be pretty much default up-counting with its clock from the system PLL
    timerBase->CR1 = TIM_CR1_URS; // Set this so that updating registers via EGR will not send an update event
    timerBase->CR2 = 0;
    timerBase->SMCR = 0;

    timerBase->DIER = TIM_DIER_UDE | TIM_DIER_CC1DE | TIM_DIER_CC2DE; // These will trigger DMA requests

    timerBase->PSC = 23; // 96MHz system clock / (23 + 1) = 4MHz clock
    // timerBase->PSC = 47999; // 96MHz system clock / (23 + 1) = 4MHz clock
    timerBase->ARR = 3; // 4MHz / (3 + 1) = 1MHz clock
    timerBase->CNT = 0;

    timerBase->CCER = 0;
    timerBase->CCR1 = 1; // 50% of the way through
    timerBase->CCR2 = 3;

    timerBase->EGR =
        TIM_EGR_UG; // Update registers now, otherwise update events will fire to the DMA immediately upon enabling

    timerBase->SR = 0; // Clear all status flags

    return true;
}

int PlatformImpl_TapShiftPacket(void *privateData, const uint8_t *buffer, int bitsToShift)
{

    while (bitsToShift > 0)
    {
        const int currentBitsToProcess = (static_cast<unsigned>(bitsToShift) > DMABuffers::MaximumBitsPerBuffer)
                                             ? DMABuffers::MaximumBitsPerBuffer
                                             : bitsToShift;

        DMABuffers::SendData(buffer, currentBitsToProcess);

        bitsToShift -= currentBitsToProcess;
        buffer += 2 * (currentBitsToProcess + 7) / 8;
    }

    return 0;
}

#else

bool PlatformImpl_HasShiftPacket()
{
    return true;
}

int PlatformImpl_TapShiftPacket(void *privateData, const uint8_t *buffer, int bitsToShift)
{
    auto gpioController = reinterpret_cast<GPIO_TypeDef *>(rjPortAddress);
    constexpr uint32_t tckSetMask = (1 << rjTck.pin);
    constexpr uint32_t tckClearMask = (0x1'0000 << rjTck.pin);
    constexpr uint32_t tdiSetMask = (1 << rjTdi.pin);
    constexpr uint32_t tdiClearMask = (0x1'0000 << rjTdi.pin);
    constexpr uint32_t tmsSetMask = (1 << rjTms.pin);
    constexpr uint32_t tmsClearMask = (0x1'0000 << rjTms.pin);
    constexpr uint32_t tdoInputMask = (1 << rjTdo.pin);

    for (; bitsToShift > 0; bitsToShift -= 8)
    {
        uint8_t tdi = buffer[0];
        uint8_t tms = buffer[1];
        uint8_t tdo = 0;

        buffer += 2;

        const int totalBitsThisShift = (bitsToShift > 8) ? 8 : bitsToShift;

        for (int bitsLeft = totalBitsThisShift; bitsLeft > 0; --bitsLeft, tdi >>= 1, tms >>= 1)
        {
            uint32_t bsrr = tckClearMask;
            bsrr |= ((tdi & 1) != 0) ? tdiSetMask : tdiClearMask;
            bsrr |= ((tms & 1) != 0) ? tmsSetMask : tmsClearMask;

            gpioController->BSRR = bsrr;

            // just *some* delay, because most JTAG ports can't toggle at 50-100MHz
            // I need to tune this better, because different STM32s will run at
            // different speeds.  Maybe I should put this into a KConfig option?
            for (int delay = 3; delay > 0; --delay)
            {
                __NOP();
            }

            gpioController->BSRR = tckSetMask;

            tdo >>= 1;
            tdo |= ((gpioController->IDR & tdoInputMask) != 0) ? 0x80 : 0x00;
        }

        tdo >>= (8 - totalBitsThisShift);

        PlatformImpl_TransmitData(privateData, &tdo, 1);
    }

    return 0;
}

#endif
}

#endif