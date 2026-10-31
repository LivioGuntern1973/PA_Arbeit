#include "Pump.h"


Pump::Pump(PinName pwm_pin,
                   PinName direction_pin,
                   float voltage_min,
                   float voltage_max) : m_Thread(osPriorityNormal),
                                        m_motor(pwm_pin, direction_pin, 1.0f, 1.0f, voltage_max, voltage_min/voltage_max)
{

    // initialise control signals
    m_velocity = voltage_max;
    m_accelerationStart = voltage_max * 0.5f;
    m_accelerationStop = voltage_max * 10.0f;

    m_motor.enableMotionPlanner();

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

void Pump::start(void)
{
    m_motor.setMaxAcceleration(m_accelerationStart);
    m_motor.setVelocity(m_velocity);
}

void Pump::stop(void)
{
    m_motor.setMaxAcceleration(m_accelerationStop);
    m_motor.setVelocity(0.0f);
}


void Pump::setVelocity(float velocity)
{
    m_velocity = velocity;
}
float Pump::getVelocity() const
{
    return m_velocity;
}


void Pump::setAccelerationStart(float acceleration)
{
    m_accelerationStart = acceleration;
}
float Pump::getAccelerationStart() const
{
    return m_accelerationStart;
}


void Pump::setAccelerationStop(float acceleration)
{
    m_accelerationStop = acceleration;
}
float Pump::getAccelerationStop() const
{
    return m_accelerationStop;
}


void Pump::setVoltageMin(float voltage_min)
{
    m_voltageMin = voltage_min;
}
float Pump::getVoltageMin() const
{
    return m_voltageMin;
}


void Pump::threadTask()
{
    while (true) {
        ThisThread::flags_wait_any(m_ThreadFlag); // wait until the ticker starts the next cycle
        // insert code for cyclic work
        float val = m_motor.getPWM();
        printf("%f\n", val);
    }
}
void Pump::sendThreadFlag()
{
    // set the thread flag to trigger the thread task
    m_Thread.flags_set(m_ThreadFlag);
}
