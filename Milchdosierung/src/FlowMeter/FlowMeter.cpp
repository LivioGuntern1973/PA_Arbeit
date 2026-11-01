#include "FlowMeter.h"

FlowMeter::FlowMeter(PinName i2c_sda_pin,
                     PinName i2c_scl_pin,
                     uint16_t i2c_address,
                     uint16_t i2c_baudrate)
    : m_i2c(i2c_sda_pin, i2c_scl_pin),
      m_i2cAddress(i2c_address),
      m_Thread(osPriorityNormal)
{
    m_i2c.frequency(i2c_baudrate);

    // Messwerte initialisieren
    m_flowRate = 0.0f;
    m_temperaturePt1000 = 0.0f;
    m_temperatureSoS = 0.0f;
    m_speedOfSound = 0.0f;
    m_VolumeLongTerm = 0.0f;
    m_VolumeShortTerm = 0.0f;
    m_bubbleDetectionLevel = 0;

    // Thread starten
    m_Thread.start(callback(this, &FlowMeter::threadTask));

    // Thread zyklisch für die Messwertverarbeitung aufwecken
    m_Ticker.attach(
        callback(this, &FlowMeter::sendThreadFlag),
        std::chrono::microseconds{PERIOD_MUS});
}

FlowMeter::~FlowMeter()
{
    m_Ticker.detach();
    m_Thread.terminate();
}

float FlowMeter::getFlowRate() const
{
    return m_flowRate;
}

float FlowMeter::getTemperaturePT1000() const
{
    return m_temperaturePt1000;
}

float FlowMeter::getSpeedOfSound() const
{
    return m_speedOfSound;
}

float FlowMeter::getTemperatureSoS() const
{
    return m_temperatureSoS;
}

void FlowMeter::setBubbleDetectionLevel(uint8_t level)
{
    // Gültigen Bereich 0 bis 4 begrenzen
    if (level > 4) {
        level = 4;
    }

    m_bubbleDetectionLevel = level;
}

uint8_t FlowMeter::getBubbleDetectionLevel() const
{
    return m_bubbleDetectionLevel;
}

void FlowMeter::threadTask()
{
    while (true) {
        ThisThread::flags_wait_any(THREAD_FLAG);

        // todo: FlowMeter-Daten per I2C lesen und Messwerte aktualisieren.
    }
}

void FlowMeter::sendThreadFlag()
{
    m_Thread.flags_set(THREAD_FLAG);
}