#ifndef MILKMATE_PIN_MAP_H
#define MILKMATE_PIN_MAP_H

#include "mbed.h"

// New MilkMate mapping, independent of the older PESBoardPinMap.h.
namespace milk_pins {
constexpr PinName MOTOR_PWM[] = {PB_15, PA_9, PB_13, PA_11};
constexpr PinName MOTOR_ENABLE[] = {PB_14, PB_10, PC_4, PC_5};
constexpr PinName MOTOR_ERROR[] = {PC_0, PC_1, PC_2, PC_3};
constexpr PinName MOTOR_DIR = PB_9;
constexpr PinName FLOW_SDA = PB_7;
constexpr PinName FLOW_SCL = PB_6;
constexpr PinName DISPLAY_MOSI = PC_12;
constexpr PinName DISPLAY_MISO = PC_11;
constexpr PinName DISPLAY_SCK = PC_10;
constexpr PinName DISPLAY_SDA = PC_9;
constexpr PinName DISPLAY_SCL = PA_8;
constexpr PinName DISPLAY_CS = PC_7;
constexpr PinName DISPLAY_DC = PC_6;
constexpr PinName DISPLAY_RESET = PB_2;
constexpr PinName RASPI_TX = PA_0;
constexpr PinName RASPI_RX = PA_1;
constexpr PinName LED_DATA = PC_8;
constexpr PinName CONSOLE_TX = PA_2;
constexpr PinName CONSOLE_RX = PA_3;
constexpr PinName FLOW_PULSE = PA_15;
constexpr PinName BUTTON[] = {PA_4, PA_6, PA_7, PB_12};
constexpr PinName SWDIO = PA_13;
constexpr PinName SWCLK = PA_14;
constexpr PinName USER_BUTTON = PC_13;
constexpr PinName USER_LED = PA_5;
} // namespace milk_pins

#endif
