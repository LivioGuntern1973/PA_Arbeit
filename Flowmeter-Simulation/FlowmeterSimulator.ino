#include <Arduino.h>
#include <Wire.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

// Durchfluss bei dauerhaftem HIGH am PWM-Eingang, in l/min.
#define MAX_FLOW_L_MIN 5.0

#if (!defined(__AVR_ATmega328P__) && !defined(__AVR_ATmega328PB__)) || F_CPU != 16000000UL
#error Dieser Sketch benoetigt einen ATmega328P oder ATmega328PB mit 16 MHz.
#endif

namespace {

constexpr uint8_t kPwmPin = 2;  // D2 = PD2 (direkter GPIO-Zugriff im Timer-Interrupt)
constexpr uint8_t kI2cAddress = 0x10;
constexpr uint8_t kFrameSize = 24;
constexpr uint32_t kUpdateIntervalUs = 50000;
constexpr uint32_t kMaxRegisterValue = 0x7FFFFFFFUL;  // Groesster positiver Int32-Wert

// 1 l/min entspricht 6000 Registereinheiten zu je 0.01 l/h.
constexpr double kScaledMaxFlow = MAX_FLOW_L_MIN * 6000.0 + 0.5;
static_assert(kScaledMaxFlow >= 1.0 && kScaledMaxFlow < 2147483648.0,
              "MAX_FLOW_L_MIN liegt ausserhalb des darstellbaren Bereichs.");
constexpr uint32_t kMaxFlowRaw = static_cast<uint32_t>(kScaledMaxFlow);

// Umrechnung von (Durchfluss-Rohwert * Mikrosekunden) in die Registereinheiten.
constexpr uint32_t kTotalizer1Divisor = 36000000UL;  // 0.1 ml
constexpr uint32_t kTotalizer2Divisor = 3600000UL;   // 0.01 ml

volatile uint16_t pwmSamples = 0;
volatile uint16_t pwmHighSamples = 0;
uint8_t sampleLfsr = 0xA5;
uint32_t windowStartUs = 0;

uint32_t previousFlowRaw = 0;
uint64_t totalVolume = 0;
uint64_t shortVolume = 0;

// Der I2C-Interrupt liest immer einen vollstaendig aufgebauten Datensatz.
uint8_t frames[2][kFrameSize] = {};
volatile uint8_t activeFrame = 0;
bool readPending = false;  // Nur in den beiden Wire-Callbacks verwendet.

void writeInt32LE(uint8_t* destination, uint32_t value) {
  for (uint8_t i = 0; i < 4; ++i) {
    destination[i] = static_cast<uint8_t>(value);
    value >>= 8;
  }
}

void receiveCommand(int length) {
  const int command = Wire.read();
  const int reserved = Wire.read();
  while (Wire.available()) {
    Wire.read();
  }
  readPending = length == 2 && command == 0x01 && reserved == 0x00;
}

void sendFrame() {
  if (readPending) {
    readPending = false;
    Wire.write(frames[activeFrame], kFrameSize);
  }
}

}  // namespace

ISR(TIMER2_COMPA_vect) {
  const bool high = (PIND & _BV(PD2)) != 0;

  // Abstaende von 16 bis 31.5 us variieren, damit der Timer bei 20 kHz
  // und anderen PWM-Frequenzen nicht an derselben PWM-Phase abtastet.
  const uint8_t feedback = (sampleLfsr & 1) ? 0xB8 : 0;
  sampleLfsr = (sampleLfsr >> 1) ^ feedback;
  OCR2A = TCNT2 + 32 + (sampleLfsr & 0x1F);

  ++pwmSamples;
  if (high) {
    ++pwmHighSamples;
  }
}

void setup() {
  pinMode(kPwmPin, INPUT);
  Wire.onReceive(receiveCommand);
  Wire.onRequest(sendFrame);
  // Der I2C-Controller gibt den SCL-Takt von 100 kHz vor.
  Wire.begin(kI2cAddress);

  // Timer2 frei laufend mit 0.5 us pro Tick. Timer0 bleibt fuer micros().
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    TCCR2B = 0;
    TCCR2A = 0;
    TCNT2 = 0;
    OCR2A = 47;
    TIFR2 = _BV(OCF2A);
    TIMSK2 = _BV(OCIE2A);
    windowStartUs = micros();
    TCCR2B = _BV(CS21);
  }
}

void loop() {
  if (micros() - windowStartUs < kUpdateIntervalUs) {
    return;
  }

  uint32_t nowUs;
  uint16_t samples;
  uint16_t highSamples;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    nowUs = micros();
    samples = pwmSamples;
    highSamples = pwmHighSamples;
    pwmSamples = 0;
    pwmHighSamples = 0;
  }
  const uint32_t elapsedUs = nowUs - windowStartUs;
  windowStartUs = nowUs;
  if (samples == 0) {
    return;
  }

  const uint32_t flowRaw = (uint64_t(kMaxFlowRaw) * highSamples + samples / 2) / samples;
  if (previousFlowRaw == 0 && flowRaw > 0) {
    shortVolume = 0;
  }
  previousFlowRaw = flowRaw;

  // Mit Rest integrieren, damit kleine Mengen auch bei kurzen Intervallen zaehlen.
  const uint64_t addedVolume = uint64_t(flowRaw) * elapsedUs;
  totalVolume = min(totalVolume + addedVolume, uint64_t(kMaxRegisterValue) * kTotalizer1Divisor);
  shortVolume = min(shortVolume + addedVolume, uint64_t(kMaxRegisterValue) * kTotalizer2Divisor);

  const uint8_t nextFrame = activeFrame ^ 1;
  writeInt32LE(frames[nextFrame] + 0, flowRaw);
  // Bytes 4 bis 15 bleiben fuer Temperatur, Signal, Luftblasen und Reserved auf 0.
  writeInt32LE(frames[nextFrame] + 16, totalVolume / kTotalizer1Divisor);
  writeInt32LE(frames[nextFrame] + 20, shortVolume / kTotalizer2Divisor);
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    activeFrame = nextFrame;
  }
}
