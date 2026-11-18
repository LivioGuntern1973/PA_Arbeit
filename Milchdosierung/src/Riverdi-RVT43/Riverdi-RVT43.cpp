/**
 * @file Riverdi-RVT43.cpp
 * @brief EVE initialization, text rendering and interrupt handling for RiverdiRVT43.
 */

#include "Riverdi-RVT43.h"

#include <chrono>
#include <cstring>

namespace {
using rtos::Kernel::Clock;
using std::chrono::milliseconds;

constexpr milliseconds BootTimeout{500};
constexpr milliseconds BootPollInterval{10};
constexpr milliseconds FrameTimeout{250};
constexpr uint16_t DisplayEnable = 1U << 15;
constexpr uint16_t PanelDriveStrength = 1U << 12;
constexpr uint16_t InterruptMask = INT_SWAP;
constexpr uint16_t EveInterruptBits = 0x01FF;
constexpr uint16_t Font = 24;
} // namespace

RiverdiRVT43::RiverdiRVT43() : m_Thread(osPriorityNormal)
{
    m_ThreadStarted = m_Thread.start(mbed::callback(this, &RiverdiRVT43::threadTask)) == osOK;
    if (!m_ThreadStarted) {
        m_State.store(State::Error);
    }
}

RiverdiRVT43::~RiverdiRVT43()
{
    if (m_ThreadStarted) {
        m_Thread.flags_set(StopFlag);
        m_Thread.join();
    }
}

bool RiverdiRVT43::setText(const char *text)
{
    if (!text) {
        return false;
    }

    std::size_t length = 0;
    for (; text[length] != '\0'; ++length) {
        if (length == MaxTextLength || text[length] < ' ' || text[length] > '~') {
            return false;
        }
    }

    mbed::ScopedLock<rtos::Mutex> lock(m_TextMutex);
    if (getState() == State::Error) {
        return false;
    }
    std::memcpy(m_Text, text, length + 1);
    m_Thread.flags_set(TextFlag);
    return true;
}

RiverdiRVT43::State RiverdiRVT43::getState() const
{
    return m_State.load();
}

void RiverdiRVT43::threadTask()
{
    if (initialize()) {
        m_State.store(State::Ready);
        while (!stopRequested()) {
            rtos::ThisThread::flags_wait_any(StopFlag | TextFlag, false);
            if (stopRequested()) {
                break;
            }

            char text[sizeof(m_Text)];
            {
                mbed::ScopedLock<rtos::Mutex> lock(m_TextMutex);
                rtos::ThisThread::flags_clear(TextFlag);
                std::memcpy(text, m_Text, sizeof(text));
            }
            if (!drawText(text)) {
                break;
            }
        }
    }

    close();
    if (!stopRequested()) {
        m_State.store(State::Error);
        // Keep the thread available for the destructor's stop signal after a fault.
        rtos::ThisThread::flags_wait_any(StopFlag);
    }
}

bool RiverdiRVT43::initialize()
{
    // App_Common_Init() opens the host and polls indefinitely. Use its panel
    // configuration with bounded boot waits and one checked open instead.
    Gpu_HalInit_t halInit{};
    if (stopRequested() || !Gpu_Hal_Init(&halInit) || !Gpu_Hal_Open(&m_Host)) {
        return false;
    }

    Gpu_Hal_Powercycle(&m_Host, TRUE);
    Gpu_HostCommand(&m_Host, GPU_EXTERNAL_OSC);
    if (!hostReady()) {
        return false;
    }
    Gpu_81X_SelectSysCLK(&m_Host, GPU_SYSCLK_72M);
    if (!hostReady()) {
        return false;
    }
    Gpu_CoreReset(&m_Host);
    if (!hostReady()) {
        return false;
    }
    Gpu_HostCommand(&m_Host, GPU_ACTIVE_M);
    if (!waitForRegister(REG_ID, 0x7C) || !waitForRegister(REG_CPURESET, 0)) {
        return false;
    }

    Gpu_Hal_Wr32(&m_Host, REG_FREQUENCY, 72000000);
    Gpu_Hal_Wr8(&m_Host, REG_PWM_DUTY, 0);
    Gpu_Hal_Wr16(&m_Host, REG_PWM_HZ, 4000);
    Gpu_Hal_Wr16(&m_Host, REG_PCLK_FREQ, DispPLCLKFREQ);
    Gpu_Hal_Wr8(&m_Host, REG_PCLK_2X, DispPCLK2x);
    Gpu_Hal_Wr16(&m_Host, REG_HCYCLE, DispHCycle);
    Gpu_Hal_Wr16(&m_Host, REG_HOFFSET, DispHOffset);
    Gpu_Hal_Wr16(&m_Host, REG_HSYNC0, DispHSync0);
    Gpu_Hal_Wr16(&m_Host, REG_HSYNC1, DispHSync1);
    Gpu_Hal_Wr16(&m_Host, REG_VCYCLE, DispVCycle);
    Gpu_Hal_Wr16(&m_Host, REG_VOFFSET, DispVOffset);
    Gpu_Hal_Wr16(&m_Host, REG_VSYNC0, DispVSync0);
    Gpu_Hal_Wr16(&m_Host, REG_VSYNC1, DispVSync1);
    Gpu_Hal_Wr16(&m_Host, REG_HSIZE, DispWidth);
    Gpu_Hal_Wr16(&m_Host, REG_VSIZE, DispHeight);
    Gpu_Hal_Wr8(&m_Host, REG_SWIZZLE, DispSwizzle);
    Gpu_Hal_Wr8(&m_Host, REG_PCLK_POL, DispPCLKPol);
    Gpu_Hal_Wr8(&m_Host, REG_CSPREAD, DispCSpread);
    Gpu_Hal_Wr8(&m_Host, REG_DITHER, DispDither);
    Gpu_Hal_Wr16(&m_Host, REG_GPIOX_DIR, DisplayEnable);
    Gpu_Hal_Wr16(&m_Host, REG_GPIOX, DisplayEnable | PanelDriveStrength);

    Gpu_Hal_Wr8(&m_Host, REG_INT_EN, 0);
    Gpu_Hal_Rd16(&m_Host, REG_INT_FLAGS); // Clear pending flags before enabling INT_N.
    if (!hostReady() || !platform_set_interrupt_callback(&m_Host, onInterrupt, this)) {
        return false;
    }
    Gpu_Hal_Wr16(&m_Host, REG_INT_MASK, InterruptMask);
    Gpu_Hal_Wr8(&m_Host, REG_INT_EN, 1);

    Gpu_ClearScreen(&m_Host);
    Gpu_Hal_Wr8(&m_Host, REG_PCLK, DispPCLK);
    if (!waitForSwap()) {
        return false;
    }
    Gpu_Hal_Wr8(&m_Host, REG_PWM_DUTY, 128);
    return hostReady();
}

void RiverdiRVT43::close()
{
    if (hostReady()) {
        Gpu_Hal_Wr8(&m_Host, REG_INT_EN, 0);
        Gpu_Hal_Wr8(&m_Host, REG_PWM_DUTY, 0);
    }
    Gpu_Hal_Close(&m_Host);
    Gpu_Hal_DeInit();
}

bool RiverdiRVT43::waitForRegister(uint32_t address, uint8_t expected)
{
    const auto deadline = Clock::now() + BootTimeout;
    while (hostReady() && !stopRequested()) {
        const uint8_t value = Gpu_Hal_Rd8(&m_Host, address);
        if (!hostReady()) {
            return false;
        }
        if (value == expected) {
            return true;
        }
        if (Clock::now() >= deadline) {
            break;
        }
        rtos::ThisThread::flags_wait_any_for(StopFlag, BootPollInterval, false);
    }
    return false;
}

bool RiverdiRVT43::drawText(const char *text)
{
    // The STM32 stores EVE's 32-bit command words in little-endian order.
    constexpr std::size_t HeaderWords = 7;
    constexpr std::size_t TextWords = (MaxTextLength + sizeof(uint32_t)) / sizeof(uint32_t);
    uint32_t commands[HeaderWords + TextWords + 2] = {
        CMD_DLSTART,
        CLEAR_COLOR_RGB(0, 0, 0),
        CLEAR(1, 1, 1),
        COLOR_RGB(255, 255, 255),
        CMD_TEXT,
        uint32_t(DispWidth / 2) | (uint32_t(DispHeight / 2) << 16),
        Font | (OPT_CENTER << 16),
    };

    const std::size_t textBytes = std::strlen(text) + 1;
    std::memcpy(commands + HeaderWords, text, textBytes);
    // Pad the terminated string to a whole word before the next command.
    const std::size_t paddedTextWords = (textBytes + sizeof(uint32_t) - 1) / sizeof(uint32_t);
    const std::size_t end = HeaderWords + paddedTextWords;
    commands[end] = DISPLAY();
    commands[end + 1] = CMD_SWAP;
    const std::size_t commandBytes = (end + 2) * sizeof(uint32_t);

    // EVE writes the block into its FIFO, including any wraparound.
    const uint16_t space = Gpu_Hal_Rd16(&m_Host, REG_CMDB_SPACE);
    if (!hostReady() || space > CMD_FIFO_SIZE - 4 || (space & 3U) || space < commandBytes) {
        return false;
    }
    Gpu_Hal_WrMem(&m_Host, REG_CMDB_WRITE, reinterpret_cast<const uint8_t *>(commands), commandBytes);
    return waitForSwap();
}

bool RiverdiRVT43::waitForSwap()
{
    const auto deadline = Clock::now() + FrameTimeout;
    while (hostReady() && !stopRequested()) {
        rtos::ThisThread::flags_wait_any_until(StopFlag | InterruptFlag, deadline, false);
        if (stopRequested()) {
            return false;
        }
        rtos::ThisThread::flags_clear(InterruptFlag);

        // Reading acknowledges the flags. Also check at the deadline if INT_N was missed.
        const uint16_t flags = Gpu_Hal_Rd16(&m_Host, REG_INT_FLAGS);
        if (!hostReady() || (flags & ~EveInterruptBits)) {
            return false;
        }
        if (flags & InterruptMask) {
            return true;
        }
        if (Clock::now() >= deadline) {
            break;
        }
    }
    return false;
}

bool RiverdiRVT43::hostReady() const
{
    return m_Host.status == GPU_HAL_OPENED;
}

bool RiverdiRVT43::stopRequested()
{
    return (rtos::ThisThread::flags_get() & StopFlag) != 0;
}

void RiverdiRVT43::onInterrupt(void *context)
{
    static_cast<RiverdiRVT43 *>(context)->m_Thread.flags_set(InterruptFlag);
}
