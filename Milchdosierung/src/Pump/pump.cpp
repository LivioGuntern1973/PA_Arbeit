#include "Pump.h"


Pump::Pump(PinName pwm_pin,
                   PinName direction_pin,
                   PinName error_pin,
                   float voltage_min,
                   float voltage_max) : m_Thread(osPriorityNormal),
                                        m_motor(pwm_pin, direction_pin, 1.0f, 60.0f, voltage_max, voltage_min/voltage_max),
                                        m_ErrorPin(error_pin)
{

    // initialise control signals
    m_velocity = voltage_max;
    m_rampTimeStart = 2;
    m_rampTimeStop = 0.5;

    m_motor.enableMotionPlanner();

    m_ErrorPin.mode(PullUp);

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
    m_motor.setMaxAcceleration(m_velocity/m_rampTimeStart);
    m_motor.setVelocity(m_velocity);
}

void Pump::stop(void)
{
    m_motor.setMaxAcceleration(m_velocity/m_rampTimeStop);
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


void Pump::setRampTimeStart(float duration)
{
    m_rampTimeStart = duration > 0.1f ? duration : 0.1f;
}
float Pump::getdurationStart() const
{
    return m_rampTimeStart;
}


void Pump::setRampTimeStop(float duration)
{
    m_rampTimeStop = duration > 0.1f ? duration : 0.1f;
}
float Pump::getRampTimeStop() const
{
    return m_rampTimeStop;
}


void Pump::setVoltageMin(float voltage_min)
{
    m_voltageMin = voltage_min;
}
float Pump::getVoltageMin() const
{
    return m_voltageMin;
}


uint16_t Pump::getError(void)
{
    if (m_ErrorPin == 0)
    {
        return 1;
    }
    return 0;
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
