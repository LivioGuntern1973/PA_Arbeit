/**
 * @file Pump.h
 * @brief This file defines the Pump class, which is used for controlling a DC motor.
 *
 * The Pump class provides functionality to control a pump
 *
 * @dependencies
 * This class relies on external components:
 * - DCMotorOpenLoop
 * - Mbed
 *
 * Usage:
 * To use the Pump class, create an instance with the required motor parameters.
 * The driver starts disabled. When using Pump directly, call enable() before
 * start() and keep it enabled until the stop ramp has completed, then call
 * disable(). Dosierung manages this sequence automatically for its owned pump.
 *
 * Example:
 * @code{.cpp}
 * #include "Pump/pump.h"
 * #include "MilkMatePinMap.h"
 *
 * int main()
 * {
 *     const float voltage_max = 12.0f; // Maximum battery voltage in volts.
 *     const float voltage_min = 1.4f;  // Minimum motor voltage in volts.
 *     Pump pump1(milk_pins::MOTOR_PWM[0], milk_pins::MOTOR_DIR,
 *                milk_pins::MOTOR_ENABLE[0], milk_pins::MOTOR_ERROR[0],
 *                voltage_min, voltage_max);
 *     pump1.enable();
 *     pump1.start();
 *     thread_sleep_for(1000);
 *     pump1.stop();
 *     while (pump1.m_motor.getPWM() != 0.0f) { thread_sleep_for(1); }
 *     pump1.disable();
 *
 *     while (true) { thread_sleep_for(100); }
 * }
 * @endcode
 *
 * @author S. Geering
 */


#ifndef PUMP_H_
#define PUMP_H_

#include "Mbed.h"
#include <DCMotorOpenLoop\DCMotorOpenLoop.h>
#include "PESBoardPinMap.h"

#include <MilkMate.h>
#include "DebounceIn.h"

    #ifndef NEW_PES_BOARD_VERSION
    #error "Pump class works only with new PES-Board version!"
    #endif

class Pump
{
public:
    /**
     * @brief Construct a new Pump object.
     *
     * @param pwm_pin The pin name for PWM control of the motor.
     * @param direction_pin The pin name for the motor's direction signal.
     * @param enable_pin The pin name for the driver's active-high enable signal (initially low).
     * @param error_pin The pin name for the driver's active-low fault signal.
     * @param voltage_min The minimum voltage for the motor (default: 0.0f).
     * @param voltage_max The maximum voltage for the motor (default: 24.0f).
     */
    explicit Pump(PinName pwm_pin,
                  PinName direction_pin,
                  PinName enable_pin,
                  PinName error_pin,
                  float voltage_min = 0.0f,
                  float voltage_max = 24.0f);

    /**
     * @brief Destroy the Pump object.
     */
    virtual ~Pump();

    /**
     * @brief Enable the pump.
     */
    void enable(void);

    /**
     * @brief Disable the pump.
     */
    void disable(void);


    /**
     * @brief Start the pump.
     *
     * Uses the start ramp.
     * Call enable() before starting the pump.
     */
    void start(void);

    /**
     * @brief Stop the pump.
     *
     * Uses the stop ramp.
     * Call disable() after stopping the pump. (when pump stands still)
     */
    void stop(void);


    /**
     * @brief Set the target velocity of the pump.
     *
     * @param velocity The target velocity in volts.
     */
    void setVelocity(float velocity);

    /**
     * @brief Get the current velocity of the pump.
     *
     * @return float The current velocity in volts.
     */
    float getVelocity() const;

    /**
     * @brief Set the duration for starting the pump.
     *
     * @param duration The duration in for starting seconds.
     */
    void setRampTimeStart(float duration);

    /**
     * @brief Get the maximum duration for starting the pump.
     *
     * @return float The maximum duration for starting in seconds.
     */
    float getdurationStart() const;

    /**
     * @brief Set the duration for stopping the pump.
     *
     * @param duration The duration for stopping in seconds.
     */
    void setRampTimeStop(float duration);

    /**
     * @brief Get the maximum duration for stopping the pump.
     *
     * @return float The maximum duration for stopping in seconds.
     */
    float getRampTimeStop() const;

    /**
     * @brief Set the minimum voltage for the motor.
     *
     * @param duration The minimum voltage for the motor in volts.
     */
    void setVoltageMin(float voltage_min);

    /**
     * @brief Get the minimum voltage for the motor.
     *
     * @return float The minimum voltage for the motor in volts.
     */
    float getVoltageMin() const;

    /**
     * @brief Check for errors.
     *
     * @return uint16_t The current error code.
     *                  0: no error
     *                  1: hardware fault (supply undervoltage, charge pump undervoltage, output overcurrent, or device overtemperature)
     */
    uint16_t getError(void);

    /**
     * @brief Motor zur Ansteuerung der Pumpe.
     */
    DCMotorOpenLoop m_motor;

private:
    static constexpr int64_t PERIOD_MUS = 50000;

    Thread m_Thread;
    Ticker m_Ticker;
    ThreadFlag m_ThreadFlag;
    DigitalOut m_EnablePin;
    DigitalIn m_ErrorPin;


    // signals
    float m_velocity;
    float m_rampTimeStart;
    float m_rampTimeStop;
    float m_voltageMin;

    void threadTask();
    void sendThreadFlag();
};

#endif /* PUMP_H_ */




