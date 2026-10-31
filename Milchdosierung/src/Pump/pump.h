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
 * Pump pump(PWM_PIN, DIRECTION_PIN, GEAR_RATIO, KN, VOLTAGE_MAX);
 * pump.setVelocity(1.5f); // command velocity to 1.5 rotations per second
 * float currentVelocity = pump.getVelocity(); // read current velocity
 * pump.setRotation(5.0f); // command rotation to 5
 * pump.getRotation(); // read current rotation
 * ```
 *
 * @author S. Geering
 */


#ifndef PUMP_H_
#define PUMP_H_

#include "Mbed.h"
#include <DCMotorOpenLoop\DCMotorOpenLoop.h>
#include "PESBoardPinMap.h"

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
     * @param voltage_max The maximum voltage for the motor (default: 12.0f).
     * @param pwm_min Minimum PWM magnitude for nonzero commands, from 0.0f to 1.0f.
     */
    explicit Pump(PinName pwm_pin,
                  PinName direction_pin,
                  float voltage_max = 12.0f,
                  float pwm_min = 0.0f);

    /**
     * @brief Destroy the Pump object.
     */
    virtual ~Pump();

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
     * @brief Get the current voltage applied to the pump.
     *
     * @return float The current voltage in volts.
     */
    float getVoltage() const;

    /**
     * @brief Set the maximum acceleration for the pump.
     *
     * @param acceleration The maximum acceleration in volts per second.
     */
    void setMaxAcceleration(float acceleration);

    /**
     * @brief Get the maximum acceleration for the pump.
     *
     * @return float The maximum acceleration in volts per second.
     */
    float getMaxAcceleration() const;

private:
    static constexpr int64_t PERIOD_MUS = 50000;

    Thread m_Thread;
    Ticker m_Ticker;
    ThreadFlag m_ThreadFlag;
    DCMotorOpenLoop m_motor;

    // motor parameters
    float m_velocity_max;
    float m_acceleration_max;

    // signals
    float m_rotation_initial;
    float m_rotation_target;
    float m_rotation_setpoint;
    float m_rotation;
    float m_velocity_target;
    float m_velocity_setpoint;
    float m_velocity;
    float m_voltage;
    float m_pwm;
    float m_pwm_min;

    void threadTask();
    void sendThreadFlag();
};

#endif /* PUMP_H_ */




