/**
 * @file Dosierung.h
 * @brief This file defines the Dosierung class, which is used for dosing milk using a DC motor and a FlowMeter.
 *
 * The Dosierung class provides functionality to dose milk or other fluids
 *
 * @dependencies
 * This class relies on external components:
 * - Pump
 * - FlowMeter
 * - Mbed
 *
 * Usage:
 * To use the Dosierung class, create an instance with the required motor parameters.
 *
 * Example:
 * ```
 * // todo
 * Dosierung Dosierung(PWM_PIN, DIRECTION_PIN, GEAR_RATIO, KN, VOLTAGE_MAX);
 * Dosierung.setVelocity(1.5f); // command velocity to 1.5 rotations per second
 * float currentVelocity = Dosierung.getVelocity(); // read current velocity
 * Dosierung.setRotation(5.0f); // command rotation to 5
 * Dosierung.getRotation(); // read current rotation
 * ```
 *
 * @author S. Geering
 */


#ifndef DOSIERUNG_H_
#define DOSIERUNG_H_

#include "Mbed.h"
#include "ThreadFlag.h"
#include "PESBoardPinMap.h"
#include <MilkMate.h>
#include <Pump/pump.h>
#include <FlowMeter/FlowMeter.h>


    #ifndef NEW_PES_BOARD_VERSION
    #error "Dosierung class works only with new PES-Board version!"
    #endif

class Dosierung
{
public:
    /**
     * @brief Construct a new Dosierung object.
     *
     * @param pwm_pin The pin name for PWM control of the motor.
     * @param voltage_max The maximum voltage for the motor (default: 12.0f).
     * @param pwm_min Minimum PWM magnitude for nonzero commands, from 0.0f to 1.0f.
     */
    explicit Dosierung(PinName pwm_pin,
                  PinName direction_pin,
                  float voltage_max = 12.0f,
                  float pwm_min = 0.0f);

    /**
     * @brief Destroy the Dosierung object.
     */
    virtual ~Dosierung();

    /**
     * @brief Set the target velocity of the Dosierung.
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

#endif /* DOSIERUNG_H_ */




