#include "DCMotorOpenLoop.h"


DCMotorOpenLoop::DCMotorOpenLoop(PinName pwm_pin,
#ifdef NEW_PES_BOARD_VERSION
                                 PinName direction_pin,
#endif
                                 float gear_ratio,
                                 float kn,
                                 float voltage_max,
                                 float pwm_min) : m_FastPWM(pwm_pin),
#ifdef NEW_PES_BOARD_VERSION
                                                       m_Direction(direction_pin),
#endif
                                                       m_Thread(osPriorityHigh1)
#if PERFORM_CHIRP_MEAS
                                                       , m_BufferedSerial(USBTX, USBRX)
#endif
{
    // motor parameters
    m_voltage_max = voltage_max;
    m_pwm_min = (pwm_min < 0.0f) ? 0.0f :
                 (pwm_min > 1.0f) ? 1.0f : pwm_min;
    m_velocity_physical_max = kn / 60.0f * voltage_max;
    m_velocity_max = m_velocity_physical_max;

    // default controller parameters, parameters adapted from gear ratio 78:1 tune
    const float k_gear = gear_ratio / 78.125f;
    setVelocityCntrl(DCMotorOpenLoop::KP * k_gear, DCMotorOpenLoop::KI * k_gear, DCMotorOpenLoop::KD * k_gear);
    if (kn > 0.0f)
        m_PIDCntrl_velocity.setParamF(60.0f / kn);
    setRotationCntrlGain();

    // iir filter

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

    // initilise motion planner, parameters adapted from gear ratio 78:1 tune
    m_enable_motion_planner = false;
    m_Motion.setPosition(0.0f);
    m_Motion.setProfileVelocity(m_velocity_max);
    m_acceleration_max = 400.0f / gear_ratio;
    setMaxAcceleration(m_acceleration_max);

#if PERFORM_GPA_MEAS
    // closed-loop measurement
    const float fMin = 1.0f;
    const float fMax = 0.99f/2.0f/TS;
    const uint16_t NfexcDes = 80;
    const float Aexc0 = 0.4f * m_velocity_max;
    const float Aexc1 = 0.5f * 0.4f * m_velocity_max; // Aexc0/fMax;
    const int   NperMin = 3;
    const float TmeasMin = 0.5f;
    const int   NmeasMin = (int)ceilf(TmeasMin/TS);
    const float Tstart = 1.0f;
    const int   Nstart = (int)ceilf(Tstart/TS);
    const float Tsweep = 0.3f;
    const int   Nsweep = (int)ceilf(Tsweep/TS);
    m_GPA.init(fMin, fMax, NfexcDes, NperMin, NmeasMin, TS, Aexc0, Aexc1, Nstart, Nsweep, true, true);
#endif

#if PERFORM_CHIRP_MEAS
    const float f0 = 0.1f;
    const float f1 = 0.99f/2.0f/TS;
    const float t1 = 60.0f;
    m_chirp.init(f0, f1, t1, TS);
    m_BufferedSerial.set_baud(2000000);
    m_BufferedSerial.set_blocking(false);
    m_timer.start();
#endif

    // start thread
    m_Thread.start(callback(this, &DCMotorOpenLoop::threadTask));

    // attach sendThreadFlag() to ticker so that sendThreadFlag() is called periodically, which signals the thread to execute
    m_Ticker.attach(callback(this, &DCMotorOpenLoop::sendThreadFlag), std::chrono::microseconds{PERIOD_MUS});
}

DCMotorOpenLoop::~DCMotorOpenLoop()
{
    m_Ticker.detach();
    m_Thread.terminate();
}

void DCMotorOpenLoop::setVelocity(float velocity)
{
    m_cntrlMode = CntrlMode::Velocity;
    m_velocity_target = velocity;
}

void DCMotorOpenLoop::setRotation(float rotation)
{
    m_cntrlMode = CntrlMode::Rotation;
    m_rotation_target = m_rotation_initial + rotation;
}

void DCMotorOpenLoop::setRotationRelative(float rotation_relative)
{
    m_cntrlMode = CntrlMode::Rotation;
    m_rotation_target = getRotation() + rotation_relative;
}

float DCMotorOpenLoop::getRotationTarget() const
{
    return m_rotation_target;
}

float DCMotorOpenLoop::getRotationSetpoint() const
{
    return m_rotation_setpoint;
}

float DCMotorOpenLoop::getRotation() const
{
    return m_rotation - m_rotation_initial;
}

float DCMotorOpenLoop::getVelocityTarget() const
{
    return m_velocity_target;
}

float DCMotorOpenLoop::getVelocitySetpoint() const
{
    return m_velocity_setpoint;
}

float DCMotorOpenLoop::getVelocity() const
{
    return m_velocity;
}

float DCMotorOpenLoop::getVoltage() const
{
    return m_voltage;
}

float DCMotorOpenLoop::getPWM() const
{
    return m_pwm;
}

void DCMotorOpenLoop::setVelocityCntrl(float kp, float ki, float kd)
{
    const float tau_f = 1.0f / (2.0f * M_PIf * 30.0f);
    const float tau_ro = 1.0f / (2.0f * M_PIf * 0.5f / (2.0f * TS));
    m_PIDCntrl_velocity.setup(kp,
                              ki,
                              kd,
                              tau_f,
                              tau_ro,
                              TS,
                              m_voltage_max * (2.0f * PWM_MIN - 1.0f),
                              m_voltage_max * (2.0f * PWM_MAX - 1.0f));
    // avoid students melting their motors
    setVelocityCntrlIntegratorLimitsPercent();
    // // print for debugging
    // printf("DCMotorOpenLoop: velocity controller parameters: kp = %.4f, ki = %.4f, kd = %.4f\n", kp, ki, kd);
}

void DCMotorOpenLoop::setVelocityCntrlIntegratorLimitsPercent(float percent_of_max)
{
    percent_of_max = percent_of_max * 0.01f;
    m_PIDCntrl_velocity.setIntegratorLimits(percent_of_max * m_voltage_max * (2.0f * PWM_MIN - 1.0f),
                                            percent_of_max * m_voltage_max * (2.0f * PWM_MAX - 1.0f));
}

void DCMotorOpenLoop::setRotationCntrlGain(float p)
{
    m_p = p;
}

void DCMotorOpenLoop::setMaxVelocity(float velocity)
{
    m_velocity_max = (velocity > m_velocity_physical_max) ? m_velocity_physical_max : velocity;
    m_Motion.setProfileVelocity(m_velocity_max);
}

float DCMotorOpenLoop::getMaxVelocity() const
{
    return m_velocity_max;
}

float DCMotorOpenLoop::getMaxPhysicalVelocity() const
{
    return m_velocity_physical_max;
}

void DCMotorOpenLoop::setMaxAcceleration(float acceleration)
{
    m_Motion.setProfileAcceleration(acceleration);
    m_Motion.setProfileDeceleration(acceleration);
}

float DCMotorOpenLoop::getMaxAcceleration() const
{
    return m_acceleration_max;
}

void DCMotorOpenLoop::enableMotionPlanner()
{
    m_enable_motion_planner = true;
}

void DCMotorOpenLoop::disableMotionPlanner()
{
    m_enable_motion_planner = false;
}

void DCMotorOpenLoop::setMotionPlannerVelocity(float velocity) {
    m_Motion.setVelocity(velocity);
}

void DCMotorOpenLoop::setMotionPlannerPosition(float position) {
    m_Motion.setPosition(position);
}

void DCMotorOpenLoop::setFastPWMPeriod_mus(int period_mus)
{
    // set the period of the PWM signal in microseconds
    m_FastPWM.period_mus(period_mus);
}

#if PERFORM_GPA_MEAS
void DCMotorOpenLoop::startGPA()
{
    if (!m_start_gpa) {
        m_start_gpa = true;
    }
}
#endif

#if PERFORM_CHIRP_MEAS
void DCMotorOpenLoop::startChirp()
{
    if (!m_start_chirp) {
        m_start_chirp = true;
        m_timer.reset();
    }
}
#endif

void DCMotorOpenLoop::threadTask()
{
    while (true) {
        ThisThread::flags_wait_any(m_ThreadFlag);

        float velocity_setpoint = 0.0f;

        switch (m_cntrlMode) {
            case CntrlMode::Rotation:
                if (m_enable_motion_planner) {
                    // Motion planner erzeugt den Sollverlauf ohne Encoder-Rueckmeldung.
                    m_Motion.incrementToPosition(m_rotation_target, TS);
                    m_rotation_setpoint = m_Motion.getPosition();
                    velocity_setpoint = m_Motion.getVelocity();
                } else {
                    m_rotation_setpoint = m_rotation_target;
                    velocity_setpoint = m_p * (m_rotation_setpoint - m_rotation);
                }
                break;

            case CntrlMode::Velocity:
                if (m_enable_motion_planner) {
                    m_Motion.incrementToVelocity(m_velocity_target, TS);
                    velocity_setpoint = m_Motion.getVelocity();
                } else {
                    velocity_setpoint = m_velocity_target;
                }
                break;

            default:
                break;
        }

        velocity_setpoint = (velocity_setpoint >  m_velocity_max) ?  m_velocity_max :
                            (velocity_setpoint < -m_velocity_max) ? -m_velocity_max :
                             velocity_setpoint;

        // Schaetzung aus dem Sollwert; die echte Motordrehung wird nicht gemessen.
        m_velocity = velocity_setpoint;
        m_velocity_setpoint = velocity_setpoint;
        m_rotation += velocity_setpoint * TS;

        // Open-loop: Sollgeschwindigkeit in Spannung umrechnen.
        float voltage = 0.0f;
        if (m_velocity_physical_max > 0.0f && m_voltage_max > 0.0f) {
            voltage = m_voltage_max * velocity_setpoint / m_velocity_physical_max;
        }
        voltage = (voltage >  m_voltage_max) ?  m_voltage_max :
                  (voltage < -m_voltage_max) ? -m_voltage_max :
                   voltage;

        float pwm_magnitude = 0.0f;
        if (m_voltage_max > 0.0f) {
            pwm_magnitude = fabs(voltage / m_voltage_max);
        }
        if (velocity_setpoint != 0.0f && pwm_magnitude < m_pwm_min) {
            pwm_magnitude = m_pwm_min;
        }

#ifdef NEW_PES_BOARD_VERSION
        const float pwm = pwm_magnitude;
        const bool direction = (velocity_setpoint > 0.0f);
        m_Direction.write(direction);
#else
        const float signed_pwm = (velocity_setpoint >= 0.0f) ? pwm_magnitude : -pwm_magnitude;
        const float pwm = 0.5f + 0.5f * signed_pwm;
#endif
        m_FastPWM.write(pwm);

        m_voltage = voltage;
        m_pwm = pwm;
    }
}
void DCMotorOpenLoop::sendThreadFlag()
{
    // set the thread flag to trigger the thread task
    m_Thread.flags_set(m_ThreadFlag);
}




