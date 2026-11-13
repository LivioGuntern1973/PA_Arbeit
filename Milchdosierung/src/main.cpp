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
    dosing.pump.setRampTimeStart(3.0f);
    dosing.pump.setRampTimeStop(0.1f);
    enable_motor = 1;
    user_button.fall(&on_user_button_pressed);

    Timer main_task_timer;
    Timer step_duration_timer;
    uint16_t step = 0;
    uint16_t step_old = 9999;
    uint16_t command_error = 0;
    unsigned int status_cycles = 0;
    main_task_timer.start();

    printf("Sensorless dosing test: press the user button to dispense 600 ml.\n");

    while (true) {
        main_task_timer.reset();
        // Consume each interrupt event once; button presses during dosing are ignored.
        const bool user_button_edge = user_button_pending.exchange(false);
        const uint16_t device_error = dosing.getState(); // Device errors, not dosing progress.

        if ((step == 2 || step == 3) && device_error != 0) {
            if (command_error == 0) {
                dosing.stop();
            }
            command_error |= device_error;
            step = 3;
        }

        switch (step) {
            case 0: // Wait for a new button press.
                if (user_button_edge) {
                    step = 1;
                }
                break;

            case 1: // Start exactly one automatic dose.
                if (device_error != 0) {
                    printf("Start rejected: device error 0x%04x. Press again after recovery.\n",
                           static_cast<unsigned int>(device_error));
                    step = 0;
                    break;
                }
                command_error = 0;
                dosing.dispenseVolume(target_volume_ml);
                step = 2;
                break;

            case 2: // Confirm motor startup before treating zero PWM as completion.
                if (dosing.pump.m_motor.getPWM() > 0.0f) {
                    step = 3;
                }
                break;

            case 3: // Dosierung plans braking and stops the pump automatically.
                if (dosing.pump.m_motor.getPWM() == 0.0f) {
                    step = 4;
                }
                break;

            case 4: // Allow the final volume update and a short pause before the next dose.
                if (std::chrono::duration_cast<std::chrono::milliseconds>(
                        step_duration_timer.elapsed_time()).count() >= 1000) {
                    printf("Dose %s: estimated volume %.1f ml, error 0x%04x.\n",
                           command_error == 0 ? "finished" : "aborted",
                           dosing.getActVolume(), static_cast<unsigned int>(command_error));
                    printf("Press the user button for another 600 ml dose.\n");
                    step = 0;
                }
                break;

            default:
                dosing.stop();
                step = 0;
                break;
        }

        const bool step_changed = step != step_old;
        if (step_changed) {
            step_duration_timer.reset();
            step_duration_timer.start();
            step_old = step;
        }

        if (step_changed || ++status_cycles >= 10) {
            printf("step=%u, estimated_volume=%.1f ml, error=0x%04x\n",
                   static_cast<unsigned int>(step), dosing.getActVolume(),
                   static_cast<unsigned int>(device_error));
            status_cycles = 0;
        }

        user_led = !user_led;
        const int elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            main_task_timer.elapsed_time()).count();
        if (elapsed_ms < main_task_period_ms) {
            thread_sleep_for(main_task_period_ms - elapsed_ms);
        } else if (elapsed_ms > main_task_period_ms) {
            printf("Warning: main task exceeded its 20 ms period.\n");
        }
    }
}

void on_user_button_pressed()
{
    user_button_pending.store(true);
}
