#include "FlowMeter.h"

namespace {

uint16_t readU16LE(const uint8_t* data)
{
    return static_cast<uint16_t>(data[0])
         | (static_cast<uint16_t>(data[1]) << 8);
}

int16_t readI16LE(const uint8_t* data)
{
    int32_t value = readU16LE(data);
    if ((value & 0x8000) != 0) {
        value -= 0x10000;
    }
    return static_cast<int16_t>(value);
}

int32_t readI32LE(const uint8_t* data)
{
    int64_t value = static_cast<uint32_t>(data[0])
                  | (static_cast<uint32_t>(data[1]) << 8)
                  | (static_cast<uint32_t>(data[2]) << 16)
                  | (static_cast<uint32_t>(data[3]) << 24);

    if ((value & 0x80000000LL) != 0) {
        value -= 0x100000000LL;
    }
    return static_cast<int32_t>(value);
}

} // namespace

FlowMeter::FlowMeter(PinName i2c_sda_pin,
                     PinName i2c_scl_pin,
                     uint16_t i2c_address,
                     uint16_t i2c_baudrate)
    : m_i2c(i2c_sda_pin, i2c_scl_pin),
      m_i2cAddress(i2c_address),
      m_Thread(osPriorityNormal)
{
    m_i2c.frequency(i2c_baudrate);

    m_flowRate = 0.0f;
    m_temperaturePt1000 = 0.0f;
    m_temperatureSoS = 0.0f;
    m_speedOfSound = 0.0f;
    m_VolumeLongTerm = 0.0f;
    m_VolumeShortTerm = 0.0f;
    m_bubbleDetectionLevel = 0;

    m_Thread.start(callback(this, &FlowMeter::threadTask));

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

float FlowMeter::getTemperatureSoS() const
{
    return m_temperatureSoS;
}

float FlowMeter::getSpeedOfSound() const
{
    return m_speedOfSound;
}

void FlowMeter::setBubbleDetectionLevel(uint8_t level)
{
    m_bubbleDetectionLevel = (level <= 4) ? level : 4;
}

uint8_t FlowMeter::getBubbleDetectionLevel() const
{
    return m_bubbleDetectionLevel;
}

void FlowMeter::threadTask()
{
    const int address = static_cast<int>(m_i2cAddress << 1);
    const char command[2] = {0x01, 0x00};
    uint8_t data[24];

    while (true) {
        ThisThread::flags_wait_any(THREAD_FLAG);

        // Messwert-Block anfordern
        if (m_i2c.write(address, command, sizeof(command), false) != 0) {
            continue;
        }

        // 24 Bytes aus dem Sensor lesen
        if (m_i2c.read(address, reinterpret_cast<char*>(data),
                       sizeof(data), false) != 0) {
            continue;
        }

        // Register 0: Flow rate, 0.01 l/h -> l/min
        m_flowRate = static_cast<float>(readI32LE(&data[0])) / 6000.0f;

        // Register 4: PT1000-Temperatur, 0.01 °C
        m_temperaturePt1000 =
            static_cast<float>(readI16LE(&data[4])) / 100.0f;

        // Register 6: Temperatur per Schallgeschwindigkeit, 0.01 °C
        m_temperatureSoS =
            static_cast<float>(readI16LE(&data[6])) / 100.0f;

        // Register 8: Schallgeschwindigkeit, 0.1 m/s
        m_speedOfSound =
            static_cast<float>(readI16LE(&data[8])) / 10.0f;

        // Register 12: Bubble detection level, gültige Werte 0 bis 4
        if (data[12] <= 4) {
            m_bubbleDetectionLevel = data[12];
        }

        // Register 16: Langzeitvolumen, 0.1 ml
        m_VolumeLongTerm =
            static_cast<float>(readI32LE(&data[16])) / 10.0f;

        // Register 20: Kurzzeitvolumen, 0.01 ml
        m_VolumeShortTerm =
            static_cast<float>(readI32LE(&data[20])) / 100.0f;
    }
}

void FlowMeter::sendThreadFlag()
{
    m_Thread.flags_set(THREAD_FLAG);
}