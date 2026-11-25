#include "mbed.h"
#include "MilkMatePinMap.h"
#include "DebounceIn.h"
#include "Dosierung/Dosierung.h"

#include <atomic>

std::atomic<bool> user_button_pending{false};
DebounceIn user_button(milk_pins::USER_BUTTON);
void on_user_button_pressed();

int main()
{
    constexpr int main_task_period_ms = 20;
    constexpr float target_volume_ml = 300.0f; // ml
    constexpr float max_flow_rate = 3.6f; // Estimated l/min at voltage_max; calibrate for the pump.

    DigitalOut user_led(milk_pins::USER_LED, 0);

    user_button.fall(&on_user_button_pressed);

    Timer main_task_timer;
    Timer step_duration_timer;
    uint16_t step = 0;
    uint16_t step_old = 9999;
    main_task_timer.start();


    const float voltage_max = 24.0f; // Maximum battery voltage in volts.
    const float voltage_min = 1.4f;  // Minimum motor voltage in volts.
    #define motor_index 1
    Pump pump1(
        milk_pins::MOTOR_PWM[motor_index], 
        milk_pins::MOTOR_DIR,
        milk_pins::MOTOR_ENABLE[motor_index], 
        milk_pins::MOTOR_ERROR[motor_index],
        voltage_min, 
        voltage_max
    );

    pump1.enable();
    thread_sleep_for(20);
    pump1.start();
    thread_sleep_for(5000);
    pump1.stop();
    while (pump1.m_motor.getPWM() != 0.0f) { thread_sleep_for(1); }
    pump1.disable();
 
    return 0;
/*
    while (true) {
        main_task_timer.reset();
        // Consume each interrupt event once; button presses during dosing are ignored.
        const bool user_button_edge = user_button_pending.exchange(false);

        switch (step) {
            case 0: // Wait for a new button press.
                if (user_button_edge) {
                    step = 1;
                }
                break;

            case 1:
                dosing.dispenseVolume(target_volume_ml);
                step = 2;
                break;

            case 2:
                if (!dosing.isPumping()) {
                    step = 3;
                }
                break;

            case 3:
                if (std::chrono::duration_cast<std::chrono::milliseconds>(
                        step_duration_timer.elapsed_time()).count() >= 1000) {
                    step = 0;
                }
                break;

            default:
                step = 0;
                break;
        }

        float flow = dosing.flowmeter->getFlowRate();
        float volume = dosing.flowmeter->getVolumeShortTerm();
        printf("%f   %f \n\r", flow, volume);

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
*/
}

void on_user_button_pressed()
{
    user_button_pending.store(true);
}
