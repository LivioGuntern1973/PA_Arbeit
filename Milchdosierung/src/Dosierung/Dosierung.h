/**
 * @file Dosierung.h
 * @brief Volume dosing with a ramp-controlled pump and a flowmeter.
 * @author S. Geering
 */

#ifndef DOSIERUNG_H_
#define DOSIERUNG_H_

#include "mbed.h"
#include "ThreadFlag.h"
#include "MilkMate.h"
#include "Pump/pump.h"
#include "FlowMeter/FlowMeter.h"

#ifndef NEW_PES_BOARD_VERSION
#error "Dosierung class works only with new PES-Board version!"
#endif

class Dosierung
{
public:
    /** Construct both devices and start the cyclic dosing task. */
    explicit Dosierung(const PumpParameters& pump_parameters,
                       const FlowMeterParameters& flowmeter_parameters);
    virtual ~Dosierung();

    /**
     * @brief Dispense a volume in ml, including the pump's braking ramp.
     *
     * Starts a new measurement and replaces any active dosing command.
     * Nonpositive or nonfinite volumes are ignored. Device errors prevent a start.
     */
    void dispenseVolume(float volume);

    /** Start continuous pumping until stop(); resets the dispensed volume. */
    void start();

    /** Stop without a braking ramp, including during automatic dosing. */
    void stop();

    /** Get the dispensed volume in ml since the last dispenseVolume() or start(). */
    float getActVolume() const;

    /**
     * @brief Set the automatic volume correction in ml.
     *
     * Positive values stop earlier, negative values stop later.
     * The effective target is max(0, requested volume - correction).
     * Nonfinite corrections are ignored; continuous pumping is unaffected.
     */
    void setStopOffsetVolume(float volume);
    float getStopOffsetVolume() const;

    /**
     * @brief Get device errors, independently of the dosing step.
     * @return Pump error in bits 0..7, FlowMeter error in bits 8..15.
     *         0x0000: no error; 0x0001: pump hardware fault;
     *         0x0100: flowmeter I2C communication failure.
     *         Both error bits may be set. Errors abort the active command.
     */
    uint16_t getState();

    /** Get the flowmeter's bubble detection level (0: no air, 4: severe air). */
    uint16_t getBubbleState() const;

    // Public devices allow callers to configure velocity, ramps and sensor parameters.
    Pump pump;
    FlowMeter flowmeter;

private:
    static constexpr int64_t PERIOD_MUS = FlowMeter::PERIOD_MUS;
    static constexpr float PERIOD_SECONDS = 1.0e-6f * static_cast<float>(PERIOD_MUS);

    enum class Step {
        Idle,
        RampUp,
        Running,
        RampDown
    };

    Thread m_Thread;
    Ticker m_Ticker;
    ThreadFlag m_ThreadFlag;
    mutable Mutex m_Mutex;
    Timer m_VolumeTimer;

    Step m_step = Step::Idle;
    bool m_automatic = false;
    float m_targetVolume = 0.0f;
    float m_actVolume = 0.0f;
    float m_previousFlowRate = 0.0f;
    float m_stopOffsetVolume = 0.0f;

    // The caller must hold m_Mutex for these helpers.
    void beginDispensing(float volume, bool automatic);
    void updateVolume();
    void updateStateMachine();
    void threadTask();
    void sendThreadFlag();
};

#endif /* DOSIERUNG_H_ */
