#include "Dosierung.h"

#include <algorithm>
#include <cmath>

constexpr int64_t Dosierung::PERIOD_MUS;
constexpr float Dosierung::PERIOD_SECONDS;

Dosierung::Dosierung(const PumpParameters& pump_parameters,
                     const FlowMeterParameters& flowmeter_parameters)
    : pump(pump_parameters.pwm_pin,
           pump_parameters.direction_pin,
           pump_parameters.error_pin,
           pump_parameters.voltage_min,
           pump_parameters.voltage_max),
      flowmeter(flowmeter_parameters.i2c_sda_pin,
                flowmeter_parameters.i2c_scl_pin,
                flowmeter_parameters.i2c_address,
                flowmeter_parameters.i2c_baudrate),
      m_Thread(osPriorityNormal)
{
    m_VolumeTimer.start();
    m_Thread.start(callback(this, &Dosierung::threadTask));
    m_Ticker.attach(callback(this, &Dosierung::sendThreadFlag),
                    std::chrono::microseconds{PERIOD_MUS});
}

Dosierung::~Dosierung()
{
    m_Ticker.detach();
    ScopedMutexLock lock(m_Mutex);
    m_Thread.terminate();
    pump.stopImmediately();
}

void Dosierung::dispenseVolume(float volume)
{
    if (!std::isfinite(volume) || volume <= 0.0f) {
        return;
    }

    ScopedMutexLock lock(m_Mutex);
    beginDispensing(volume, true);
}

void Dosierung::start()
{
    ScopedMutexLock lock(m_Mutex);
    beginDispensing(0.0f, false);
}

void Dosierung::stop()
{
    ScopedMutexLock lock(m_Mutex);
    updateVolume();
    pump.stopImmediately();
    m_automatic = false;
    if (m_step != Step::Idle) {
        m_step = Step::RampDown;
    }
}

float Dosierung::getActVolume() const
{
    ScopedMutexLock lock(m_Mutex);
    return m_actVolume;
}

void Dosierung::setStopOffsetVolume(float volume)
{
    if (std::isfinite(volume)) {
        ScopedMutexLock lock(m_Mutex);
        m_stopOffsetVolume = volume;
    }
}

float Dosierung::getStopOffsetVolume() const
{
    ScopedMutexLock lock(m_Mutex);
    return m_stopOffsetVolume;
}

uint16_t Dosierung::getState()
{
    return static_cast<uint16_t>((pump.getError() & 0x00ffU)
                                | ((flowmeter.getError() & 0x00ffU) << 8));
}

uint16_t Dosierung::getBubbleState() const
{
    return flowmeter.getBubbleDetectionLevel();
}

void Dosierung::beginDispensing(float volume, bool automatic)
{
    m_actVolume = 0.0f;
    m_targetVolume = volume;
    m_automatic = automatic;
    m_previousFlowRate = std::max(0.0f, flowmeter.getFlowRate()) * (1000.0f / 60.0f);
    m_VolumeTimer.reset();

    if (getState() != 0 || (automatic && volume <= m_stopOffsetVolume)) {
        pump.stopImmediately();
        m_step = Step::RampDown;
        m_automatic = false;
        return;
    }

    pump.start();
    m_step = Step::RampUp;
}

void Dosierung::updateVolume()
{
    const float elapsed = 1.0e-6f * static_cast<float>(m_VolumeTimer.elapsed_time().count());
    m_VolumeTimer.reset();

    const float flow = std::max(0.0f, flowmeter.getFlowRate()) * (1000.0f / 60.0f);
    if (m_step != Step::Idle) {
        // Trapezoidal integration of forward flow in ml/s, including ramp-down.
        m_actVolume += 0.5f * (m_previousFlowRate + flow) * elapsed;
    }
    m_previousFlowRate = flow;
}

void Dosierung::updateStateMachine()
{
    if (m_step != Step::Idle && getState() != 0) {
        pump.stopImmediately();
        m_automatic = false;
        m_step = Step::RampDown;
    }

    const float velocity = std::fabs(pump.m_motor.getVelocity());
    const float configuredVelocity = std::fabs(pump.getVelocity());

    switch (m_step) {
        case Step::Idle:
            break;

        case Step::RampUp:
        case Step::Running: {
            if (m_automatic) {
                const float flow = std::max(0.0f, flowmeter.getFlowRate()) * (1000.0f / 60.0f);
                // Pump::stop() brakes with a = configured velocity / stop ramp time.
                // During ramp-up, stopping takes only the current fraction of that time.
                const float stopTime = configuredVelocity > 0.0f
                    ? pump.getRampTimeStop() * velocity / configuredVelocity : 0.0f;
                const float brakingVolume = 0.5f * flow * stopTime;
                const float remainingVolume = m_targetVolume - m_stopOffsetVolume - m_actVolume;

                // Place the decision at the middle of a cycle to reduce sampling error.
                if (remainingVolume <= brakingVolume + 0.5f * flow * PERIOD_SECONDS) {
                    pump.stop();
                    m_step = Step::RampDown;
                    break;
                }
            }

            const float runningVelocity = std::min(configuredVelocity, pump.m_motor.getMaxVelocity());
            m_step = velocity + 0.01f >= runningVelocity ? Step::Running : Step::RampUp;
            break;
        }

        case Step::RampDown:
            if (velocity < 0.01f) {
                m_step = Step::Idle;
                m_automatic = false;
            }
            break;
    }
}

void Dosierung::threadTask()
{
    while (true) {
        ThisThread::flags_wait_any(m_ThreadFlag);
        ScopedMutexLock lock(m_Mutex);
        updateVolume();
        updateStateMachine();
    }
}

void Dosierung::sendThreadFlag()
{
    m_Thread.flags_set(m_ThreadFlag);
}
