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
 *
 * Example:
 * ```
 * const float voltage_max = 12.0f; // maximum voltage of battery packs
 * const float voltage_min = 1.4f;  // minimum pwm at slow speed [%]
 * Pump pump1(PB_PWM_M1, PB_DIR_M1, PUMP1_ERROR, voltage_min, voltage_max);
 * enable_motors = 1;
 * pump1.start();
 * ```
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
     * @param voltage_min The minimum voltage for the motor (default: 0.0f).
     * @param voltage_max The maximum voltage for the motor (default: 24.0f).
     */
    explicit Pump(PinName pwm_pin,
                  PinName direction_pin,
                  PinName error_pin,
                  float voltage_min = 0.0f,
                  float voltage_max = 24.0f);

    /**
     * @brief Destroy the Pump object.
     */
    virtual ~Pump();

    /**
     * @brief Start the pump.
     */
    void start(void);

    /**
     * @brief Stop the pump.
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

private:
    static constexpr int64_t PERIOD_MUS = 50000;

    Thread m_Thread;
    Ticker m_Ticker;
    ThreadFlag m_ThreadFlag;
    DCMotorOpenLoop m_motor;
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




