/**
 * @file MilkMate.h
 * @brief Shared types for the MilkMate devices.
 */

#ifndef MILKMATE_H_
#define MILKMATE_H_

#include "mbed.h"
#include <cstdint>

/** Constructor parameters for a Pump owned by Dosierung. */
struct PumpParameters {
    PinName pwm_pin;
    PinName direction_pin;
    PinName error_pin;
    float voltage_min = 0.0f;
    float voltage_max = 24.0f;
};

/** Constructor parameters for a FlowMeter owned by Dosierung. */
struct FlowMeterParameters {
    PinName i2c_sda_pin;
    PinName i2c_scl_pin;
    uint16_t i2c_address; // Unshifted, 7-bit I2C address.
    uint32_t i2c_baudrate = 100000;
};

enum class DeviceState {
    idle = 0,
    busy,
    error
};

#endif /* MILKMATE_H_ */

