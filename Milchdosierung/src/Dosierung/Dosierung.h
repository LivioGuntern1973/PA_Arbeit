/**
 * @file Dosierung.h
 * @brief This file defines the Dosierung class for dosing milk or other liquids.
 *
 * @author S. Geering
 */

#ifndef DOSIERUNG_H_
#define DOSIERUNG_H_

#include "mbed.h"
#include "ThreadFlag.h"
#include "Pump/pump.h"
#include "FlowMeter/FlowMeter.h"
#include <cstdint>
#include <memory>

#ifndef NEW_PES_BOARD_VERSION
#error "Dosierung class works only with new PES-Board version!"
#endif

/**
 * @brief Constructor parameters for the pump owned by Dosierung.
 */
struct PumpParameters {
    PinName pwm_pin;             ///< Pin for the pump motor's PWM signal.
    PinName direction_pin;       ///< Pin for the pump motor's direction signal.
    PinName enable_pin;          ///< Pin for the pump driver's enable signal.
    PinName error_pin;           ///< Pin for the pump driver's active-low fault signal.
    float voltage_min = 0.0f;    ///< Minimum motor voltage in volts (default: 0.0f).
    float voltage_max = 24.0f;   ///< Maximum motor voltage in volts (default: 24.0f).
    float max_flow_rate = 5.0f;  ///< Estimated flow in l/min at voltage_max (default: 5.0f).
};

/**
 * @brief Constructor parameters for the flowmeter owned by Dosierung.
 */
struct FlowMeterParameters {
    PinName i2c_sda_pin;                 ///< Pin for the I2C SDA signal.
    PinName i2c_scl_pin;                 ///< Pin for the I2C SCL signal.
    uint16_t i2c_address;                ///< Unshifted, 7-bit I2C address of the sensor.
    uint32_t i2c_baudrate = 100000;       ///< I2C clock frequency in Hz (default: 100000).
};

/**
 * @brief Dose milk or other liquids using a pump and an optional I2C flowmeter.
 *
 * The Dosierung class provides automatic volume dosing and continuous pumping
 * for manual operation. Its cyclic task uses the same period as FlowMeter
 * (currently 20 ms) and controls the idle, ramp-up, running and ramp-down steps.
 * During automatic dosing, the braking point is estimated from the current
 * flow rate, motor velocity and configured pump braking ramp. Volume is
 * integrated from the measured forward flow, including the braking ramp.
 * Without a flowmeter, forward flow is estimated linearly from motor velocity
 * and PumpParameters::max_flow_rate, referenced to the constructor's voltage_max.
 * Pump uses a motor constant of 60 rpm/V, so its motor ramp velocity in rotations
 * per second numerically equals the voltage command. Reducing the motor velocity
 * limit therefore also reduces the estimated flow. Ramp-up and ramp-down use
 * the same dosing logic in both operating modes.
 *
 * A positive volume correction increases the requested dose; a negative
 * correction decreases it. Device errors stop the active command using the
 * pump's minimum braking ramp. The bubble detection level is available for
 * reporting and does not influence dosing.
 *
 * @par Dependencies
 * This class relies on external components:
 * - Pump: Motor control with configurable start and stop ramps.
 * - FlowMeter (optional): I2C flow measurement, communication status and bubble detection.
 * - Mbed: Threads, ticker, timer and mutex synchronization.
 * - ThreadFlag: Periodic task signaling.
 *
 * @par Usage
 * Create PumpParameters and FlowMeterParameters with the required pins and
 * device settings, then pass both to the Dosierung constructor. Configure
 * the public pump and flowmeter objects before starting a command. Configure
 * the pump only while idle and serialize external configuration calls; its
 * public motor and configuration methods do not use the dosing mutex. Enable
 * the motor driver externally and keep the Dosierung object alive while
 * dosing. Use dispenseVolume() for a measured dose, or start() and stop()
 * for continuous pumping. Use isPumping() to wait until ramp-down and the final
 * volume update have completed. Check getState() separately for device errors.
 * Command and measurement methods are called from thread context.
 * To operate without a flowmeter, pass only PumpParameters. No sensor object
 * or sensor task is created. When present, the public sensor is accessed
 * through the flowmeter pointer, for example flowmeter->getFlowRate().
 *
 * @par Example with a flowmeter
 * Replace the example I2C address 0x08 with the sensor's configured address.
 * @code{.cpp}
 * #include "Dosierung/Dosierung.h"
 * #include "MilkMatePinMap.h"
 *
 * int main()
 * {
 *     const PumpParameters pump_parameters = {
 *         milk_pins::MOTOR_PWM[0], milk_pins::MOTOR_DIR, milk_pins::MOTOR_ERROR[0], 1.4f, 12.0f
 *     };
 *     const FlowMeterParameters flowmeter_parameters = {
 *         milk_pins::FLOW_SDA, milk_pins::FLOW_SCL, 0x08
 *     };
 *
 *     DigitalOut enable_motor(milk_pins::MOTOR_ENABLE[0], 0);
 *     Dosierung dosing(pump_parameters, flowmeter_parameters);
 *     enable_motor = 1;
 *     dosing.dispenseVolume(600.0f); // Dose 6 dl = 600 ml.
 *     while (dosing.isPumping()) { thread_sleep_for(20); }
 *     printf("Volume: %.1f ml, error: 0x%04x\n", dosing.getActVolume(),
 *            static_cast<unsigned int>(dosing.getState()));
 *
 *     while (true) { thread_sleep_for(100); }
 * }
 * @endcode
 *
 * @par Example without a flowmeter
 * Uses the default maximum flow rating of 5 l/min; volume is estimated.
 * @code{.cpp}
 * #include "Dosierung/Dosierung.h"
 * #include "MilkMatePinMap.h"
 *
 * int main()
 * {
 *     const PumpParameters pump_parameters = {
 *         milk_pins::MOTOR_PWM[0], milk_pins::MOTOR_DIR, milk_pins::MOTOR_ERROR[0], 1.4f, 12.0f
 *     };
 *     DigitalOut enable_motor(milk_pins::MOTOR_ENABLE[0], 0);
 *     Dosierung dosing(pump_parameters);
 *     enable_motor = 1;
 *     dosing.dispenseVolume(600.0f); // Estimate and dose 6 dl = 600 ml.
 *     while (dosing.isPumping()) { thread_sleep_for(20); }
 *     printf("Estimated volume: %.1f ml, error: 0x%04x\n", dosing.getActVolume(),
 *            static_cast<unsigned int>(dosing.getState()));
 *
 *     while (true) { thread_sleep_for(100); }
 * }
 * @endcode
 */
class Dosierung
{
public:
    /**
     * @brief Construct a new Dosierung object.
     *
     * Constructs the public devices and starts the cyclic dosing task in idle.
     *
     * @param pump_parameters Pins and voltage limits passed to the Pump constructor.
     * @param flowmeter_parameters Pins, I2C address and clock frequency passed to the FlowMeter constructor.
     */
    explicit Dosierung(const PumpParameters& pump_parameters,
                       const FlowMeterParameters& flowmeter_parameters);

    /**
     * @brief Construct a Dosierung object without a flowmeter.
     *
     * Estimates flow from motor velocity and the maximum flow rating.
     * No FlowMeter object, I2C connection or sensor task is created.
     *
     * @param pump_parameters Pump pins, voltage limits and maximum flow in l/min.
     */
    explicit Dosierung(const PumpParameters& pump_parameters);

    /**
     * @brief Destroy the Dosierung object.
     *
     * Detaches the ticker, wakes and joins the dosing task, then commands a pump
     * stop using the minimum braking ramp. Waits for zero PWM before the owned
     * devices are destroyed. Must be called from another thread, with no other
     * callers still accessing this object.
     */
    virtual ~Dosierung();

    /**
     * @brief Start automatic volume dosing, accounting for the pump's braking ramp.
     *
     * Starts a new measurement and replaces any active dosing command.
     * Nonpositive or nonfinite volumes are ignored. Device errors prevent a start.
     * Rejected commands preserve the current command and recorded volume.
     * The effective target is the requested volume plus the configured correction,
     * limited to a minimum of zero and captured when the command is accepted.
     * A nonfinite sum is rejected. A zero effective target commands a pump stop
     * without starting the motor.
     * Without a flowmeter, a nonpositive or nonfinite maximum flow rating
     * prevents automatic dosing. Pump voltage, velocity limit and configured
     * forward velocity must be finite and positive; ramp times must be finite.
     *
     * @param volume The requested volume in milliliters, before volume correction.
     */
    void dispenseVolume(float volume);

    /**
     * @brief Start continuous pumping for manual operation.
     *
     * Resets the measured volume and replaces any active dosing command.
     * Uses the pump's configured velocity and start ramp. Volume correction
     * does not apply. Pumping continues until stop() or a device error;
     * an existing device error or invalid pump configuration prevents a start
     * without resetting the recorded volume.
     */
    void start();

    /**
     * @brief Stop the pump using its minimum braking ramp.
     *
     * Cancels automatic or continuous pumping. Temporarily selects the pump's
     * minimum braking ramp (0.1 s), issues the stop command and restores the
     * previous ramp setting. Volume measurement continues during ramp-down.
     */
    void stop();

    /**
     * @brief Check whether a pump operation is still active.
     *
     * Becomes true as soon as a start() or dispenseVolume() command is accepted,
     * including the time before the motor's first ramp update. Remains true
     * during ramp-up, running and ramp-down, including braking after stop() or
     * a device error. Becomes false when the cyclic task confirms standstill
     * and completes the final volume update, normally within one 20 ms cycle.
     * Works with and without a flowmeter. This method is mutex-protected and
     * must be called from thread context. Use getState() to query device errors.
     *
     * @return bool Whether the pump operation is still active.
     * @retval true A pump operation is starting, running or braking.
     * @retval false No pump operation is active.
     */
    bool isPumping() const;

    /**
     * @brief Get the measured or estimated volume of the current or most recent command.
     *
     * A new valid dispenseVolume() or start() command resets the measurement.
     * The measurement includes forward flow during ramp-up and ramp-down and
     * remains available after dosing finishes.
     * Without a flowmeter, volume is estimated from motor velocity and the
     * configured maximum flow rating instead of measured flow.
     *
     * @return float The dispensed volume in milliliters.
     */
    float getActVolume() const;

    /**
     * @brief Set the automatic volume correction in ml.
     *
     * Positive values dispense more volume, negative values dispense less.
     * The next automatic command uses max(0, requested volume + correction).
     * Nonfinite corrections are ignored; active commands and continuous pumping
     * are unaffected.
     *
     * @param volume The additive volume correction in milliliters.
     */
    void setVolumeCorrection(float volume);

    /**
     * @brief Get the automatic volume correction.
     *
     * @return float The correction in milliliters; positive adds volume and negative subtracts volume.
     */
    float getVolumeCorrection() const;

    /**
     * @brief Get device errors, independently of the dosing step.
     *
     * The cyclic task stops the active command on device errors. A cleared
     * error does not automatically restart pumping. Bubble detection is
     * excluded from the error status.
     * Without a flowmeter, only pump errors are reported.
     *
     * @return uint16_t The combined status: pump error in bits 0..7 and flowmeter error in bits 8..15.
     * @retval 0x0000 No device error.
     * @retval 0x0001 Pump hardware fault.
     * @retval 0x0100 Flowmeter I2C communication failure.
     * @retval 0x0101 Both pump and flowmeter errors are present.
     */
    uint16_t getState();

    /**
     * @brief Get the flowmeter's bubble detection level.
     *
     * Reporting only; the bubble level does not affect dosing or device error status.
     *
     * @return uint16_t The current bubble detection level.
     * @retval 0 No air, or no flowmeter configured.
     * @retval 1 Small amount of air.
     * @retval 2 Medium amount of air.
     * @retval 3 High amount of air.
     * @retval 4 Severe amount of air or completely air.
     */
    uint16_t getBubbleState() const;

    /**
     * @brief Public pump object for configuring velocity, ramps and motor parameters.
     */
    Pump pump;

    /**
     * @brief Optional public flowmeter for accessing sensor parameters and measurements.
     *
     * Null in sensorless mode. Use the arrow operator to access a configured
     * sensor. Ownership remains with Dosierung.
     */
    const std::unique_ptr<FlowMeter> flowmeter;

private:
    /**
     * @brief Construct the devices and task shared by both operating modes.
     *
     * @param pump_parameters Pump pins, voltage limits and maximum flow in l/min.
     * @param flowmeter_parameters Sensor configuration, or nullptr for sensorless operation.
     */
    Dosierung(const PumpParameters& pump_parameters,
              const FlowMeterParameters* flowmeter_parameters);

    static constexpr int64_t PERIOD_MUS = FlowMeter::PERIOD_MUS; ///< Task period in microseconds.
    static constexpr float PERIOD_SECONDS = 1.0e-6f * static_cast<float>(PERIOD_MUS); ///< Task period in seconds.

    /**
     * @brief Internal steps of the dosing state machine.
     */
    enum class Step {
        Idle,       ///< No active dosing command.
        RampUp,     ///< Pump accelerates using its configured start ramp.
        Running,    ///< Pump operates at its configured velocity.
        RampDown    ///< Pump decelerates while the remaining volume is measured.
    };

    Thread m_Thread;             ///< Thread executing the cyclic dosing task.
    Ticker m_Ticker;             ///< Ticker triggering the task at the flowmeter period.
    ThreadFlag m_ThreadFlag;     ///< Flag used to signal the dosing thread.
    mutable Mutex m_Mutex;      ///< Protects dosing commands, state and volume measurements.
    Timer m_VolumeTimer;        ///< Measures elapsed time between volume updates.
    const float m_maxPumpVoltage; ///< Constructor voltage_max in volts, defining the rated flow.
    const float m_maxPumpFlowRate; ///< Rated forward flow in l/min at m_maxPumpVoltage; zero if invalid.

    Step m_step = Step::Idle;            ///< Current state-machine step.
    bool m_shutdownRequested = false;   ///< Requests cooperative task exit; protected by m_Mutex.
    bool m_automatic = false;            ///< True for volume dosing; false for continuous pumping.
    float m_targetVolume = 0.0f;         ///< Effective automatic target in ml, including correction.
    float m_actVolume = 0.0f;            ///< Integrated forward volume in ml.
    float m_previousFlowRate = 0.0f;     ///< Previous forward flow sample in ml/s.
    float m_volumeCorrection = 0.0f;     ///< Additive automatic volume correction in ml.
    std::chrono::microseconds m_previousVolumeTime{0}; ///< Timestamp of the previous flow sample.

    /**
     * @brief Initialize a command and start the pump when permitted.
     *
     * @param volume The requested volume in ml before correction; unused in continuous mode.
     * @param automatic True for volume dosing, false for continuous pumping.
     * @pre The caller holds m_Mutex.
     */
    void beginDispensing(float volume, bool automatic);

    /**
     * @brief Command a stop with the minimum ramp and restore the previous ramp setting.
     *
     * @pre The caller holds m_Mutex.
     */
    void stopPumpWithMinimumRamp();

    /**
     * @brief Get the forward flow used for volume integration and braking.
     *
     * @return float The forward flow rate in milliliters per second.
     * @pre The caller holds m_Mutex.
     */
    float getFlowRateMlPerSecond() const;

    /**
     * @brief Update the dispensed volume using trapezoidal flow integration.
     *
     * @pre The caller holds m_Mutex.
     */
    void updateVolume();

    /**
     * @brief Evaluate device errors, the braking point and dosing-step transitions.
     *
     * @pre The caller holds m_Mutex.
     */
    void updateStateMachine();

    /**
     * @brief Wait for ticker signals, execute cyclic work and exit on shutdown.
     */
    void threadTask();

    /**
     * @brief Signal the dosing thread from the ticker interrupt.
     */
    void sendThreadFlag();
};

#endif /* DOSIERUNG_H_ */
