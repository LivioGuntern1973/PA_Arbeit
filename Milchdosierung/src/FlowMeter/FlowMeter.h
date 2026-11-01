/**
 * @file FlowMeter.h
 * @brief This file defines the FlowMeter class, which is used for controlling a Allengra FlowMeter over I2C.
 *
 * The FlowMeter class provides functionality to control a FlowMeter
 *
 * @dependencies
 * This class relies on external components:
 * - Mbed
 * - I2C
 *
 * Usage:
 * To use the FlowMeter class, create an instance with the required parameters.
 *
 * Example:
 * ```
 * FlowMeter FlowMeter(PWM_PIN, DIRECTION_PIN, GEAR_RATIO, KN, VOLTAGE_MAX);
 * FlowMeter.setVelocity(1.5f); // command velocity to 1.5 rotations per second
 * float currentVelocity = FlowMeter.getVelocity(); // read current velocity
 * FlowMeter.setRotation(5.0f); // command rotation to 5
 * FlowMeter.getRotation(); // read current rotation
 * ```
 *
 * @author S. Geering
 */


#ifndef FLOW_METER_H_
#define FLOW_METER_H_

#include "Mbed.h"
#include "PESBoardPinMap.h"

    #ifndef NEW_PES_BOARD_VERSION
    #error "FlowMeter class works only with new PES-Board version!"
    #endif

class FlowMeter
{
public:
    /**
     * @brief Construct a new FlowMeter object.
     *
     * @param i2c_sda_pin The pin name for I2C SDA.
     * @param i2c_scl_pin The pin name for I2C SCL.
     * @param i2c_address The I2C Address of the FlowMeter.
     * @param i2c_baudrate The I2C baudrate.
     */
    explicit FlowMeter(PinName i2c_sda_pin,
                       PinName i2c_scl_pin,
                       uint16_t i2c_address,
                       uint16_t i2c_baudrate);

    /**
     * @brief Destroy the FlowMeter object.
     */
    virtual ~FlowMeter();

    /**
     * @brief Get the current flow rate.
     *
     * @return float flow rate in liters per minute.
     */
    float getFlowRate() const;

    /**
     * @brief Get the physical measured temperature with PT1000.
     *
     * @return float The current temperature with PT1000 in °C.
     */
    float getTemperaturePT1000() const;

    /**
     * @brief Get the Physical measured temperature with speed of sound.
     *
     * @return float The current temperature with speed of sound in °C.
     */
    float getTemperatureSoS() const;

    /**
     * @brief Get the speed of sound.
     *
     * @return float The speed of sound in meter per second.
     */
    float getSpeedOfSound() const;

    /**
     * @brief Set the bubble detection level.
     *
     * @param level The bubble detection level
     *              0 – No air 
     *              1 – Small amount of air
     *              2 – Medium amount of air 
     *              3 – High amount of air
     *              4 – Severe / completely air
     */
    void setBubbleDetectionLevel(uint8_t level);

    /**
     * @brief Get the bubble detection level.
     *
     * @return uint8_t The bubble detection level.
     *                 0 – No air 
     *                 1 – Small amount of air
     *                 2 – Medium amount of air 
     *                 3 – High amount of air
     *                 4 – Severe / completely air
     */
    uint8_t getBubbleDetectionLevel() const;

private:
    static constexpr int64_t PERIOD_MUS = 10000;
    static constexpr uint32_t THREAD_FLAG = 1U;
    I2C m_i2c;
    uint16_t m_i2cAddress;

    Thread m_Thread;
    Ticker m_Ticker;

    // motor parameters
    float m_velocity_max;
    float m_acceleration_max;

    // signals
    float m_flowRate;
    float m_temperaturePt1000;
    float m_temperatureSoS;
    float m_speedOfSound;
    float m_VolumeLongTerm;
    float m_VolumeShortTerm;
    uint8_t m_bubbleDetectionLevel;

    void threadTask();
    void sendThreadFlag();
};

#endif /* FLOW_METER_H_ */




