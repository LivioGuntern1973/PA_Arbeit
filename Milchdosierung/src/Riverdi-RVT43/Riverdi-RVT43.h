/**
 * @file Riverdi-RVT43.h
 * @brief Threaded driver for the Riverdi RVT43HLBNWC00-B display.
 */
#ifndef RIVERDI_RVT43_H_
#define RIVERDI_RVT43_H_

#include "mbed.h"
#include <atomic>
#include <cstddef>

#include "platform.h"

/**
 * @brief Display a centered line of text on the Riverdi RVT43HLBNWC00-B.
 *
 * A dedicated thread opens the EVE host, initializes the panel and draws text.
 * The PA10 interrupt handler wakes that thread. The thread reads and clears
 * the EVE interrupt flags to confirm a display-list swap. Boot and frame waits
 * have time limits. On a display fault the thread closes the host and reports
 * State::Error.
 *
 * @par Dependencies
 * - Riverdi EVE library with the EVE_4 and IPS_43 panel configuration.
 * - Mbed host layer for 8-bit SPI in mode 0 at 1 MHz and GPIO.
 * - MilkMatePinMap.h for the display pins, including PA10 for INT_N.
 * - Mbed OS threads, flags and mutex synchronization.
 *
 * @par Usage
 * Create one RiverdiRVT43 instance for the connected display in thread context.
 * Call setText() during initialization or after getState() reports State::Ready.
 * Keep the object alive while the display is in use. To retry after a fault,
 * destroy the existing object and create a new one.
 *
 * @par Example
 * @code{.cpp}
 * #include "Riverdi-RVT43/Riverdi-RVT43.h"
 * #include <cstdio>
 *
 * int main()
 * {
 *     RiverdiRVT43 display;
 *     while (display.getState() == RiverdiRVT43::State::Initializing) {
 *         thread_sleep_for(10);
 *     }
 *
 *     if (display.getState() == RiverdiRVT43::State::Ready && display.setText("MilkMate ready")) {
 *         while (display.getState() == RiverdiRVT43::State::Ready) {
 *             thread_sleep_for(100);
 *         }
 *     }
 *
 *     std::printf("Display error\n");
 *     return 1;
 * }
 * @endcode
 */
class RiverdiRVT43
{
public:
    /** @brief Status of the display worker and the connected panel. */
    enum class State {
        Initializing, ///< The display thread is starting or configuring the panel.
        Ready,        ///< The initial frame has been confirmed and the panel accepts updates.
        Error         ///< Starting the thread, initializing the panel or drawing a frame failed.
    };

    static constexpr std::size_t MaxTextLength = 31; ///< Maximum number of printable ASCII characters.

    /**
     * @brief Start the display thread and initialize the panel asynchronously.
     *
     * Use getState() to check the result of initialization.
     * @pre Call from thread context after Mbed OS has started.
     */
    RiverdiRVT43();

    /**
     * @brief Request shutdown and join the display thread.
     *
     * The display thread closes the EVE host, which detaches the interrupt callback.
     * @pre Call from another thread after all other callers have finished using this object.
     */
    ~RiverdiRVT43();

    /**
     * @brief Queue a centered line of text for the display thread.
     *
     * Copies the text under a mutex before returning. A newer request replaces
     * any pending text. Use getState() to check for display errors after queuing.
     *
     * @param[in] text Null-terminated string of up to MaxTextLength printable ASCII
     * characters (0x20 to 0x7E). An empty string clears the screen.
     * @retval true The text was queued for drawing.
     * @retval false The input was null or invalid, or the display is in State::Error.
     * @pre Call from thread context.
     */
    bool setText(const char *text);

    /**
     * @brief Read the current display state atomically.
     *
     * @retval State::Initializing Panel initialization is in progress.
     * @retval State::Ready Panel initialization succeeded.
     * @retval State::Error A fault occurred and remains reported for this instance.
     */
    State getState() const;

private:
    static constexpr uint32_t StopFlag = 1U << 0;      ///< Request cooperative shutdown.
    static constexpr uint32_t TextFlag = 1U << 1;      ///< A text update is pending.
    static constexpr uint32_t InterruptFlag = 1U << 2; ///< PA10 detected a falling edge on INT_N.

    Gpu_Hal_Context_t m_Host{};                       ///< Zero-initialized EVE context owned by the display thread.
    rtos::Mutex m_TextMutex;                          ///< Protects the pending text and its notification.
    char m_Text[MaxTextLength + 1]{};                  ///< Pending text, including its null terminator.
    std::atomic<State> m_State{State::Initializing};  ///< Status shared with callers.
    rtos::Thread m_Thread;                            ///< Thread that owns the EVE host and processes display updates.
    bool m_ThreadStarted = false;                     ///< Tracks whether the destructor needs to join the thread.

    /** @brief Initialize the panel, process text updates and close the host on fault or shutdown. */
    void threadTask();

    /**
     * @brief Open the host, boot EVE, configure panel timings and display the initial frame.
     *
     * Registers the PA10 callback before enabling the EVE swap interrupt.
     * @return Whether the panel is ready for text updates.
     */
    bool initialize();

    /** @brief Disable interrupts and backlight when the host is healthy, then release the host. */
    void close();

    /**
     * @brief Poll an EVE boot register until it matches or the wait ends.
     * @param address Address of the 8-bit EVE register.
     * @param expected Register value required to continue booting.
     * @return Whether a successful read returned the expected value before timeout or shutdown.
     */
    bool waitForRegister(uint32_t address, uint8_t expected);

    /**
     * @brief Submit a display list for the text and wait for its frame swap.
     * @param text Validated, null-terminated copy of the queued text.
     * @return Whether the frame swap was confirmed.
     */
    bool drawText(const char *text);

    /**
     * @brief Wait for and acknowledge the EVE display-list swap interrupt.
     * @return Whether INT_SWAP was read successfully within the frame wait.
     */
    bool waitForSwap();

    /**
     * @brief Check whether the host is open and healthy between transfers.
     * @return Whether the current HAL status is GPU_HAL_OPENED.
     */
    bool hostReady() const;

    /**
     * @brief Check the display thread's shutdown flag.
     * @return Whether shutdown has been requested.
     * @pre Call from the display thread.
     */
    static bool stopRequested();

    /**
     * @brief Signal the display thread from the PA10 interrupt handler.
     * @param context Pointer to the RiverdiRVT43 instance registered with the host.
     */
    static void onInterrupt(void *context);
};

#endif /* RIVERDI_RVT43_H_ */
