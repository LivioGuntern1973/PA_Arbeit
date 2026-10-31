#include "mbed.h"
#include "PESBoardPinMap.h"

// drivers
#include "DebounceIn.h"
#include "FastPWM.h"
#include "DCMotorOpenLoop.h"

bool user_button_latch = false; // this variable will be toggled via the user button (blue button) and decides whether to execute the main task or not

// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(BUTTON1);   // create DebounceIn to evaluate the user button
void toggle_user_button_state_fcn(); // custom function which is getting executed when user button gets pressed, definition at the end

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
    const float voltage_max = 12.0f; // maximum voltage of battery packs, adjust this to
    const float gear_ratio_M1 = 25.0f; // gear ratio
    const float kn_M1 = 180.0f / 12.0f;  // motor constant [rpm/V]
    const float pwm_min_M1 = 0.12f;  // minimum pwm at slow speed [%]
    DCMotorOpenLoop motor_M1(PB_PWM_M1, PB_DIR_M1,gear_ratio_M1, kn_M1, voltage_max, pwm_min_M1);
    // enable the motion planner for smooth movement
    motor_M1.enableMotionPlanner();
    motor_M1.setMaxVelocity(motor_M1.getMaxPhysicalVelocity() * 0.5f);
    motor_M1.setMaxAcceleration(0.5f);
    enable_motors = 1;


    main_task_timer.start();


    while (true) {
        main_task_timer.reset();


        if (user_button_latch) {
            motor_M1.setVelocity(0.5f);
        } else {
            motor_M1.setVelocity(-2.0f);
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
}
