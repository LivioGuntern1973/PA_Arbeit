#include "ClassTemplate.h"


ClassTemplate::ClassTemplate(PinName pwm_pin,
                   PinName direction_pin,
                   float voltage_max,
                   float pwm_min) : m_Thread(osPriorityNormal),
                                    m_motor(pwm_pin, direction_pin, 1.0f, 1.0f, voltage_max, pwm_min)
{
 


    // initialise control signals
    m_rotation_initial = 0.0f;
    m_rotation_target = m_rotation_initial;
    m_rotation_setpoint = m_rotation_initial;
    m_rotation = m_rotation_initial;
    m_velocity_target = 0.0f;
    m_velocity_setpoint = 0.0f;
    m_velocity = 0.0f;
    m_voltage = 0.0f;
    m_pwm = 0.0f;

    m_acceleration_max = 400.0f;
    setMaxAcceleration(m_acceleration_max);

    // start thread
    m_Thread.start(callback(this, &ClassTemplate::threadTask));

    // attach sendThreadFlag() to ticker so that sendThreadFlag() is called periodically, which signals the thread to execute
    m_Ticker.attach(callback(this, &ClassTemplate::sendThreadFlag), std::chrono::microseconds{PERIOD_MUS});
}

ClassTemplate::~ClassTemplate()
{
    m_Ticker.detach();
    m_Thread.terminate();
}

void ClassTemplate::setVelocity(float velocity)
{
    m_velocity_setpoint = velocity;
}

float ClassTemplate::getVelocity() const
{
    return m_motor.getVoltage();
}

float ClassTemplate::getVoltage() const
{
    return m_motor.getVoltage();
}

void ClassTemplate::setMaxAcceleration(float acceleration)
{
    m_motor.setMaxAcceleration(acceleration);
}

float ClassTemplate::getMaxAcceleration() const
{
    return m_motor.getMaxAcceleration();
}


void ClassTemplate::threadTask()
{
    while (true) {
        ThisThread::flags_wait_any(m_ThreadFlag); // wait until the ticker starts the next cycle
        // insert code for cyclic work
    }
}
void ClassTemplate::sendThreadFlag()
{
    // set the thread flag to trigger the thread task
    m_Thread.flags_set(m_ThreadFlag);
}
