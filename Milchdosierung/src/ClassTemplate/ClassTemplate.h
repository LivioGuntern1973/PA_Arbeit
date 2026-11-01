/**
 * @file ClassTemplate.h
 * @brief This file defines the ClassTemplate class, which is used for controlling a DC motor.
 *
 * The ClassTemplate class provides functionality to control a ClassTemplate
 *
 * @dependencies
 * This class relies on external components:
 * - DCMotorOpenLoop
 * - Mbed
 *
 * Usage:
 * To use the ClassTemplate class, create an instance with the required motor parameters.
 *
 * Example:
 * ```
 * ClassTemplate classTemplate(PWM_PIN, DIRECTION_PIN, GEAR_RATIO, KN, VOLTAGE_MAX);
 * ClassTemplate.setVelocity(1.5f); // command velocity to 1.5 rotations per second
 * float currentVelocity = ClassTemplate.getVelocity(); // read current velocity
 * ClassTemplate.setRotation(5.0f); // command rotation to 5
 * ClassTemplate.getRotation(); // read current rotation
 * ```
 *
 * @author S. Geering
 */


#ifndef ClassTemplate_H_
#define ClassTemplate_H_

#include "Mbed.h"
#include "ThreadFlag.h"
#include "PESBoardPinMap.h"

    #ifndef NEW_PES_BOARD_VERSION
    #error "ClassTemplate class works only with new PES-Board version!"
    #endif

class ClassTemplate
{
public:
    /**
     * @brief Construct a new ClassTemplate object.
     *
     * @param pwm_pin The pin name for PWM control of the motor.
     * @param voltage_max The maximum voltage for the motor (default: 12.0f).
     * @param pwm_min Minimum PWM magnitude for nonzero commands, from 0.0f to 1.0f.
     */
    explicit ClassTemplate(PinName pwm_pin,
                  PinName direction_pin,
                  float voltage_max = 12.0f,
                  float pwm_min = 0.0f);

    /**
     * @brief Destroy the ClassTemplate object.
     */
    virtual ~ClassTemplate();

    /**
     * @brief Set the target velocity of the ClassTemplate.
     *
     * @param velocity The target velocity in volts.
     */
    void setVelocity(float velocity);

private:
    static constexpr int64_t PERIOD_MUS = 50000;

    Thread m_Thread;
    Ticker m_Ticker;
    ThreadFlag m_ThreadFlag;

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

#endif /* ClassTemplate_H_ */




