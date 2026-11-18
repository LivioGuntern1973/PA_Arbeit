#include "mbed.h"
#include "MilkMatePinMap.h"
#include "DebounceIn.h"
#include "Dosierung/Dosierung.h"
#include "Riverdi-RVT43/Riverdi-RVT43.h"

#include <atomic>

// Riverdi EVE C API with the Mbed SPI/GPIO host layer.
extern "C" {
#include "platform.h"
#include "App_Common.h"
}

std::atomic<bool> user_button_pending{false};
DebounceIn user_button(milk_pins::USER_BUTTON);
void on_user_button_pressed();

int main()
{
    constexpr int main_task_period_ms = 20;
    constexpr float target_volume_ml = 600.0f; // 6 dl = 600 ml.
    constexpr float voltage_min = 1.4f;
    constexpr float voltage_max = 12.0f;
    constexpr float max_flow_rate = 5.0f; // Estimated l/min at voltage_max; calibrate for the pump.

    DigitalOut user_led(milk_pins::USER_LED, 0);
    DigitalOut enable_motor(milk_pins::MOTOR_ENABLE[0], 0);

    const PumpParameters pump_parameters = {
        milk_pins::MOTOR_PWM[0], milk_pins::MOTOR_DIR, milk_pins::MOTOR_ERROR[0],
        voltage_min, voltage_max, max_flow_rate
    };
    Dosierung dosing(pump_parameters); // No FlowMeter object, sensor task or I2C connection.
    enable_motor = 1;
    user_button.fall(&on_user_button_pressed);

    Timer main_task_timer;
    Timer step_duration_timer;
    uint16_t step = 0;
    uint16_t step_old = 9999;
    main_task_timer.start();

    
    RiverdiRVT43 display;
    while (display.getState() == RiverdiRVT43::State::Initializing) {
    thread_sleep_for(10);

    while (true) {
        main_task_timer.reset();
        // Consume each interrupt event once; button presses during dosing are ignored.
        const bool user_button_edge = user_button_pending.exchange(false);

        
        // Display Beispiel
        if (display.getState() == RiverdiRVT43::State::Ready && display.setText("MilkMate ready")) {
            while (display.getState() == RiverdiRVT43::State::Ready) {
                thread_sleep_for(100);
            }
        }
        std::printf("Display error\n");
        return 1;
        

        if (step != step_old) {
            step_duration_timer.reset();
            step_duration_timer.start();
            step_old = step;
        }

        user_led = !user_led;
        const int elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            main_task_timer.elapsed_time()).count();
        if (elapsed_ms < main_task_period_ms) {
            thread_sleep_for(main_task_period_ms - elapsed_ms);
        }
    }
}

void on_user_button_pressed()
{
    user_button_pending.store(true);
}
