#include "Pump.h"


Pump::Pump(PinName pwm_pin,
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
    m_Thread.start(callback(this, &Pump::threadTask));

    // attach sendThreadFlag() to ticker so that sendThreadFlag() is called periodically, which signals the thread to execute
    m_Ticker.attach(callback(this, &Pump::sendThreadFlag), std::chrono::microseconds{PERIOD_MUS});
}

Pump::~Pump()
{
    m_Ticker.detach();
    m_Thread.terminate();
}

void Pump::setVelocity(float velocity)
{
    m_velocity_setpoint = velocity;
}

float Pump::getVelocity() const
{
    return m_motor.getVoltage();
}

float Pump::getVoltage() const
{
    return m_motor.getVoltage();
}

void Pump::setMaxAcceleration(float acceleration)
{
    m_motor.setMaxAcceleration(acceleration);
}

float Pump::getMaxAcceleration() const
{
    return m_motor.getMaxAcceleration();
}


void Pump::threadTask()
{
    while (true) {
        ThisThread::flags_wait_any(m_ThreadFlag);

    }
}
void Pump::sendThreadFlag()
{
    // set the thread flag to trigger the thread task
    m_Thread.flags_set(m_ThreadFlag);
}
