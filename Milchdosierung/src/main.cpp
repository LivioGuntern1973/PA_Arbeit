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
    constexpr float max_flow_rate = 3.6f; // Estimated l/min at voltage_max; calibrate for the pump.

    DigitalOut user_led(milk_pins::USER_LED, 0);

    const FlowMeterParameters flowmeter_parameters = {
        milk_pins::FLOW_SDA, milk_pins::FLOW_SCL, 0x10
    };

    const PumpParameters pump_parameters = {
        milk_pins::MOTOR_PWM[0],
//        milk_pins::MOTOR_DIR, // für Test mit Milk-Board
//        milk_pins::MOTOR_ENABLE[0], // für Test mit Milk-Board
        PB_14, // DIR-Pin für Test am PES board (einfach unbenutzter Pin)
        PB_9, // Enable-Pin für Test am PES board
        milk_pins::MOTOR_ERROR[0],
        voltage_min, 
        voltage_max, 
        max_flow_rate
    };
    Dosierung dosing(pump_parameters, flowmeter_parameters); // No FlowMeter object, sensor task or I2C connection.
//    Dosierung dosing(pump_parameters); // With FlowMeter object, sensor task and I2C connection.
#if 1
    FlowMeter flowmeter(milk_pins::FLOW_SDA, milk_pins::FLOW_SCL, 0x10, 100000);
#endif

    user_button.fall(&on_user_button_pressed);

    Timer main_task_timer;
    Timer step_duration_timer;
    uint16_t step = 0;
    uint16_t step_old = 9999;
    main_task_timer.start();

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

            case 1: // Start exactly one automatic dose.
                dosing.dispenseVolume(target_volume_ml);
                step = 2;
                break;

            case 2: // Wait for the operation, including ramp-up and braking, to finish.
                if (!dosing.isPumping()) {
                    step = 3;
                }
                break;

            case 3: // Pause before accepting the next dose; the final volume is available.
                if (std::chrono::duration_cast<std::chrono::milliseconds>(
                        step_duration_timer.elapsed_time()).count() >= 1000) {
                    step = 0;
                }
                break;

            default:
                step = 0;
                break;
        }

        float flow = flowmeter.getFlowRate();
        float volume1 = flowmeter.getTotalizer1();
        float volume2 = flowmeter.getTotalizer2();
        printf("%f   %f    %f \n\r", flow, volume1, volume2);

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
