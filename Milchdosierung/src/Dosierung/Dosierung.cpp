/**
 * @file Dosierung.cpp
 * @brief Cyclic volume integration and pump control for Dosierung.
 */

#include "Dosierung.h"

#include <algorithm>
#include <cmath>

constexpr int64_t Dosierung::PERIOD_MUS;
constexpr float Dosierung::PERIOD_SECONDS;

Dosierung::Dosierung(const PumpParameters& pump_parameters,
                     const FlowMeterParameters& flowmeter_parameters)
    : Dosierung(pump_parameters, &flowmeter_parameters)
{
}

Dosierung::Dosierung(const PumpParameters& pump_parameters)
    : Dosierung(pump_parameters, nullptr)
{
}

Dosierung::Dosierung(const PumpParameters& pump_parameters,
                     const FlowMeterParameters* flowmeter_parameters)
    : pump(pump_parameters.pwm_pin,
           pump_parameters.direction_pin,
           pump_parameters.enable_pin,
           pump_parameters.error_pin,
           pump_parameters.voltage_min,
           pump_parameters.voltage_max),
      flowmeter(flowmeter_parameters
          ? new FlowMeter(flowmeter_parameters->i2c_sda_pin,
                          flowmeter_parameters->i2c_scl_pin,
                          flowmeter_parameters->i2c_address,
                          flowmeter_parameters->i2c_baudrate)
          : nullptr),
      m_Thread(osPriorityNormal),
      m_maxPumpVoltage(pump_parameters.voltage_max),
      m_maxPumpFlowRate(std::isfinite(pump_parameters.max_flow_rate * (1000.0f / 60.0f))
          ? std::max(0.0f, pump_parameters.max_flow_rate) : 0.0f)
{
    m_VolumeTimer.start();
    m_Thread.start(callback(this, &Dosierung::threadTask));
    m_Ticker.attach(callback(this, &Dosierung::sendThreadFlag),
                    std::chrono::microseconds{PERIOD_MUS});
}

Dosierung::~Dosierung()
{
    m_Ticker.detach();
    {
        ScopedMutexLock lock(m_Mutex);
        m_shutdownRequested = true;
        sendThreadFlag(); // Keep the worker alive until its wake-up has been sent.
    }
    m_Thread.join(); // Never terminate a worker while it may own a device mutex.

    {
        ScopedMutexLock lock(m_Mutex);
        stopPumpWithMinimumRamp();
    }
    // Pump and motor tasks must remain alive until the zero PWM write completes.
    while (pump.m_motor.getPWM() != 0.0f) {
        ThisThread::sleep_for(std::chrono::milliseconds{1});
    }
    m_VolumeTimer.stop();
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
    stopPumpWithMinimumRamp();
    m_automatic = false;
    if (m_step != Step::Idle) {
        m_step = Step::RampDown;
    }
}

bool Dosierung::isPumping() const
{
    ScopedMutexLock lock(m_Mutex);
    return m_step != Step::Idle;
}

float Dosierung::getActVolume() const
{
    ScopedMutexLock lock(m_Mutex);
    return m_actVolume;
}

void Dosierung::setVolumeCorrection(float volume)
{
    if (std::isfinite(volume)) {
        ScopedMutexLock lock(m_Mutex);
        m_volumeCorrection = volume;
    }
}

float Dosierung::getVolumeCorrection() const
{
    ScopedMutexLock lock(m_Mutex);
    return m_volumeCorrection;
}

uint16_t Dosierung::getState()
{
    const uint16_t flowmeterError = flowmeter ? flowmeter->getError() : 0;
    return static_cast<uint16_t>((pump.getError() & 0x00ffU)
                                | ((flowmeterError & 0x00ffU) << 8));
}

uint16_t Dosierung::getBubbleState() const
{
    return flowmeter ? flowmeter->getBubbleDetectionLevel() : 0;
}

void Dosierung::beginDispensing(float volume, bool automatic)
{
    const float target = automatic ? std::max(0.0f, volume + m_volumeCorrection) : 0.0f;
    const float configuredVelocity = pump.getVelocity();
    const float velocityLimit = pump.m_motor.getMaxVelocity();
    if (getState() != 0 || !std::isfinite(target)
        || !std::isfinite(m_maxPumpVoltage) || m_maxPumpVoltage <= 0.0f
        || !std::isfinite(configuredVelocity) || configuredVelocity <= 0.0f
        || !std::isfinite(velocityLimit) || velocityLimit <= 0.0f
        || !std::isfinite(pump.getdurationStart())
        || !std::isfinite(pump.getRampTimeStop())
        || (automatic && !flowmeter && m_maxPumpFlowRate <= 0.0f)) {
        return; // Validate before changing either the active command or its volume.
    }

    m_actVolume = 0.0f;
    m_targetVolume = target;
    m_automatic = automatic;
    m_previousFlowRate = getFlowRateMlPerSecond();
    m_previousVolumeTime = m_VolumeTimer.elapsed_time();

    if (automatic && target == 0.0f) {
        stopPumpWithMinimumRamp();
        m_step = Step::RampDown;
        m_automatic = false;
        return;
    }

    pump.start();
    m_step = Step::RampUp;
}

void Dosierung::stopPumpWithMinimumRamp()
{
    const float previousRampTime = pump.getRampTimeStop();
    pump.setRampTimeStop(0.1f); // Minimum supported by the existing Pump API.
    pump.stop(); // Applies the acceleration immediately; restoring the parameter does not change it.
    pump.setRampTimeStop(previousRampTime);
}

float Dosierung::getFlowRateMlPerSecond() const
{
    if (flowmeter) {
        const float flow = flowmeter->getFlowRate();
        return std::isfinite(flow) && flow > 0.0f ? flow * (1000.0f / 60.0f) : 0.0f;
    }

    // Pump's 60 rpm/V motor makes the ramp velocity numerically equal to volts.
    const float velocity = pump.m_motor.getVelocity();
    const float velocityRatio = std::isfinite(velocity) && m_maxPumpVoltage > 0.0f
        ? std::max(0.0f, std::min(1.0f, velocity / m_maxPumpVoltage)) : 0.0f;
    return m_maxPumpFlowRate * velocityRatio * (1000.0f / 60.0f);
}

void Dosierung::updateVolume()
{
    const float flow = getFlowRateMlPerSecond();
    const auto now = m_VolumeTimer.elapsed_time();
    const float elapsed = 1.0e-6f * static_cast<float>((now - m_previousVolumeTime).count());
    m_previousVolumeTime = now;
    if (m_step != Step::Idle) {
        // Trapezoidal integration of forward flow in ml/s, including ramp-down.
        m_actVolume += 0.5f * (m_previousFlowRate + flow) * elapsed;
    }
    m_previousFlowRate = flow;
}

void Dosierung::updateStateMachine()
{
    if (m_step != Step::Idle && getState() != 0
        && (m_step != Step::RampDown || m_automatic)) {
        stopPumpWithMinimumRamp();
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
                const float flow = m_previousFlowRate; // Use the same sample as integration.
                // Pump::stop() brakes with a = configured velocity / stop ramp time.
                // During ramp-up, stopping takes only the current fraction of that time.
                const float stopTime = configuredVelocity > 0.0f
                    ? pump.getRampTimeStop() * velocity / configuredVelocity : 0.0f;
                const float brakingVolume = 0.5f * flow * stopTime;
                const float remainingVolume = m_targetVolume - m_actVolume;

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
            if (velocity == 0.0f) {
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
        if (m_shutdownRequested) {
            return;
        }
        updateVolume();
        updateStateMachine();
    }
}

void Dosierung::sendThreadFlag()
{
    m_Thread.flags_set(m_ThreadFlag);
}
