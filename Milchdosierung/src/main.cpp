#include "mbed.h"
#include "PESBoardPinMap.h"
#include "MilkMatePinMap.h"

// drivers
#include "DebounceIn.h"
#include "FastPWM.h"
#include <DCMotorOpenLoop/DCMotorOpenLoop.h>
#include <Pump/Pump.h>

bool user_button_latch = false; // this variable will be toggled via the user button (blue button)
bool user_button_edge = false; // this variable will be set for one cycle when pressing the blue button
bool manual_mode = false; //
#define AND_MANUAL_MODE(transition) ((transition) && (manual_mode == 0 or user_button_edge))

// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(BUTTON1);   // create DebounceIn to evaluate the user button
void toggle_user_button_state_fcn(); // custom function which is getting executed when user button gets pressed

// main runs as an own thread
int main()
{
    // attach button fall function address to user button object
    user_button.fall(&toggle_user_button_state_fcn);

    // while loop gets executed every main_task_period_ms milliseconds, this is a simple approach to repeatedly execute main
    const int main_task_period_ms = 20; // define main task period time in ms e.g. 20 ms, therefore the main task will run 50 times per second
    Timer main_task_timer;              // create Timer object which we use to run the main task every main_task_period_ms


    DigitalOut user_led(LED1); // led on nucleo board
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS); // create object to enable power electronics for the dc motors


    // motor M1
    const float voltage_max = 12.0f; // maximum voltage of battery packs
    const float voltage_min = 1.4f;  // minimum pwm at slow speed [%]
    Pump pump1(PB_PWM_M1, PB_DIR_M1, PC_0, voltage_min, voltage_max);
    pump1.setRampTimeStart(3.0);
    pump1.setRampTimeStop(0.0);
    enable_motors = 1;

    // state machine
    Timer step_duration_timer;
    uint16_t step = 0;
    uint16_t step_old = 9999; // set above 0 to detect a step change at startup


    main_task_timer.start();

    while (true) {
        main_task_timer.reset();
        
        switch (step) {
            case 0:
                

                if (user_button_edge)
                {
                    step = 1;
                }
                break;
            case 1:
                pump1.start();
                step = 2;
                break;
            case 2:
                if (AND_MANUAL_MODE(duration_cast<milliseconds>(step_duration_timer.elapsed_time()).count() >= 3000))
                {
                    step = 3;
                }
                break;
            case 3:
                pump1.stop();
                step = 4;
                break;
            case 4:
                if (AND_MANUAL_MODE(duration_cast<milliseconds>(step_duration_timer.elapsed_time()).count() >= 1000))
                {
                    step = 5;
                }
                break;
            default:
                step = 0;
        }

        //printf("%d\n", pump1.getError());


        // detect a change of the step
        if (step != step_old)
        {
            step_duration_timer.reset();
            step_duration_timer.start();
            step_old = step;
        }
        
        printf("%d\n", step);


        // user button edge
        if (user_button_edge)
        {
            user_button_edge = 0;
        }
        
        // toggling the user led
        user_led = !user_led;
        // read timer and make the main thread sleep for the remaining time span (non blocking)
        int main_task_elapsed_time_ms = duration_cast<milliseconds>(main_task_timer.elapsed_time()).count();
        if (main_task_period_ms - main_task_elapsed_time_ms < 0)
            printf("Warning: Main task took longer than main_task_period_ms\n");
        else
            thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
    }
}

void toggle_user_button_state_fcn()
{
    // toggle user_button_latch if the button was pressed
    user_button_latch = !user_button_latch;
    user_button_edge = 1;
}
