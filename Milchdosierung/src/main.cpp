#include "mbed.h"

#include <chrono>
#include <cstdint>
#include <cstdio>

/*
 * Automatischer Pin-Mapping-/Mbed-OS-Konflikttest fuer das NUCLEO-F446RE.
 *
 * Dieses Programm ist fuer ein nacktes Nucleo ohne angeschlossene Aktoren
 * oder Module gedacht. Es initialisiert alle im Pinmapping verwendeten Pins,
 * sendet auf UART/I2C und gibt regelmaessig den Laufzustand auf dem ST-Link
 * Virtual COM Port aus.
 *
 * I2C: Es wird nur ein Adressbyte an 0x20 gesendet. Ohne Slave ist NACK
 * normal; ein Datenbyte wird nur gesendet, wenn ein Slave ACK gibt.
 * WS2811: PC8 gibt normales 1-kHz-PWM aus, kein WS2811-Protokoll.
 */

#ifndef MOTOR_ENABLE_ACTIVE_LEVEL
#define MOTOR_ENABLE_ACTIVE_LEVEL 1
#endif

#if (MOTOR_ENABLE_ACTIVE_LEVEL != 0) && (MOTOR_ENABLE_ACTIVE_LEVEL != 1)
#error "MOTOR_ENABLE_ACTIVE_LEVEL muss 0 oder 1 sein"
#endif

static constexpr int kMotorEnableInactiveLevel =
    (MOTOR_ENABLE_ACTIVE_LEVEL == 1) ? 0 : 1;
static constexpr int kI2cProbeAddress7Bit = 0x20;

// Mbed-stdio verwendet auf diesem Target standardmaessig PA2/PA3 (USART2/VCP).
// PA2 = MCU-TX zum ST-Link, PA3 = MCU-RX vom ST-Link.

// UART4: ALT0 ist wichtig; PC10/PC11 ohne ALT0 waeren USART3.
static BufferedSerial raspberryPiUart(PC_10_ALT0, PC_11_ALT0, 115200);
static BufferedSerial displayUart(PC_6, PC_7, 115200);

// I2C-Konstruktor-Reihenfolge: SDA, SCL.
static I2C flowmeterI2c(PB_7, PB_6);
static I2C displayI2c(PC_9, PA_8);

// Motor-Ausgaenge; Enable startet im inaktiven Pegel.
static DigitalOut motorEnable(PB_9, kMotorEnableInactiveLevel);
static DigitalOut motor1Dir(PB_14, 0);
static DigitalOut motor2Dir(PB_10, 1);
static DigitalOut motor3Dir(PC_4, 0);
static DigitalOut motor4Dir(PC_5, 1);

// Alle vier Motor-PWM-Pins liegen auf TIM1.
static PwmOut motor1Pwm(PB_15); // TIM1_CH3N
static PwmOut motor2Pwm(PA_9);  // TIM1_CH2
static PwmOut motor3Pwm(PB_13); // TIM1_CH1N
static PwmOut motor4Pwm(PA_11); // TIM1_CH4

// Pin-/AF-Test fuer den WS2811-Pin; erzeugt kein WS2811-Datenprotokoll.
static PwmOut ws2811PinTest(PC_8); // TIM3_CH3

// Fehlerpins ohne internen Pull, da der Treiberausgang den Pegel vorgeben soll.
static DigitalIn motor1Fault(PC_0, PullNone);
static DigitalIn motor2Fault(PC_1, PullNone);
static DigitalIn motor3Fault(PC_2, PullNone);
static DigitalIn motor4Fault(PC_3, PullNone);

// Taster-Eingaenge mit Pull-up; ein Taster schaltet nach GND.
static DigitalIn button1(PA_4, PullUp);
static DigitalIn button2(PA_6, PullUp);
static DigitalIn button3(PA_7, PullUp);
static DigitalIn button4(PB_12, PullUp);

static volatile uint32_t flowPulseCount = 0;
static bool directionPhase = false;
static unsigned int pwmPhase = 0;

static void setupFlowmeterCapture()
{
    // PA5 als AF1/TIM2_CH1 fuer Input Capture konfigurieren.
    // TIM5 bleibt dem Mbed-us_ticker vorbehalten.
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    (void)RCC->APB1ENR;

    GPIOA->MODER = (GPIOA->MODER & ~(3UL << (5U * 2U))) | (2UL << (5U * 2U));
    GPIOA->PUPDR &= ~(3UL << (5U * 2U));
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(0xFUL << (5U * 4U))) |
                    (1UL << (5U * 4U)); // AF1 = TIM2

    TIM2->CR1 = 0;
    TIM2->CR2 = 0;
    TIM2->SMCR = 0;
    TIM2->DIER = 0;
    TIM2->CCER = 0;
    TIM2->CCMR1 = 0;
    TIM2->PSC = 0;
    TIM2->ARR = 0xFFFFFFFFUL;
    TIM2->CNT = 0;

    // CC1S=01: TI1 Input Capture, steigende Flanke, Filter aus.
    TIM2->CCMR1 = TIM_CCMR1_CC1S_0;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0;
    TIM2->DIER = TIM_DIER_CC1IE;
    TIM2->CCER = TIM_CCER_CC1E;

    NVIC_ClearPendingIRQ(TIM2_IRQn);
    NVIC_SetPriority(TIM2_IRQn, 5U);
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->CR1 = TIM_CR1_CEN;
}

extern "C" void TIM2_IRQHandler(void)
{
    const uint32_t status = TIM2->SR;
    if ((status & TIM_SR_CC1IF) != 0U) {
        // Lesen von CCR1 quittiert CC1IF.
        const volatile uint32_t captured = TIM2->CCR1;
        (void)captured;
        ++flowPulseCount;
    }
}

static void setMotorPwmPattern(unsigned int phase)
{
    // TIM1 hat fuer alle vier Kanaele dieselbe Frequenz; Duty-Cycles variieren.
    static const float dutyPatterns[][4] = {
        {0.10f, 0.25f, 0.40f, 0.55f},
        {0.20f, 0.35f, 0.50f, 0.65f},
        {0.30f, 0.45f, 0.60f, 0.75f},
    };
    const float *duty = dutyPatterns[phase % 3U];
    motor1Pwm.write(duty[0]);
    motor2Pwm.write(duty[1]);
    motor3Pwm.write(duty[2]);
    motor4Pwm.write(duty[3]);
}

static int probeI2cAddress(I2C &bus)
{
    // Adressphase ohne Register-/Datenbyte. NACK ist ohne Slave erwartbar.
    bus.start();
    const int ack = bus.write(kI2cProbeAddress7Bit << 1);
    bus.stop();
    return ack;
}

static const char *i2cResultName(int result)
{
    // Mbed I2C::write(int) liefert 0=NACK, 1=ACK, 2=Timeout.
    switch (result) {
        case 0: return "NACK/kein Slave";
        case 1: return "ACK";
        case 2: return "TIMEOUT";
        default: return "I2C-Fehler";
    }
}

static void sendUartTestMessages()
{
    static const char piMessage[] = "PINMAP TEST UART4 PC10 TX\r\n";
    static const char displayMessage[] = "PINMAP TEST USART6 PC6 TX\r\n";
    (void)raspberryPiUart.write(piMessage, sizeof(piMessage) - 1U);
    (void)displayUart.write(displayMessage, sizeof(displayMessage) - 1U);
}

static void reportState(int i2c1Ack, int i2c3Ack)
{
    const uint32_t pulses = flowPulseCount;
    printf("FAULT PC0..3=%d%d%d%d | BTN PA4/PA6/PA7/PB12=%d%d%d%d (0=gedr.) | ",
           motor1Fault.read(), motor2Fault.read(), motor3Fault.read(), motor4Fault.read(),
           button1.read(), button2.read(), button3.read(), button4.read());
    printf("TIM2_CH1=%lu | I2C1=%s(%d) I2C3=%s(%d) | TIM1-PWM-Muster=%u\r\n",
           static_cast<unsigned long>(pulses),
           i2cResultName(i2c1Ack), i2c1Ack,
           i2cResultName(i2c3Ack), i2c3Ack,
           pwmPhase % 3U);
    fflush(stdout);
}

static void readAndReportUart(BufferedSerial &port, const char *name)
{
    char buffer[32];
    const ssize_t count = port.read(buffer, sizeof(buffer));
    if (count > 0) {
        printf("%s RX: ", name);
        for (ssize_t i = 0; i < count; ++i) {
            printf("%02X ", static_cast<unsigned char>(buffer[i]));
        }
        printf("\r\n");
    }
}

int main()
{
    raspberryPiUart.set_blocking(false);
    displayUart.set_blocking(false);

    flowmeterI2c.frequency(10000);
    displayI2c.frequency(10000);

    // I2C-Leitungen auf dem nackten Board haben keine externen Pull-ups.
    // Interne Pull-ups helfen fuer den unbelasteten Pin-/Bus-Initialisierungstest.
    GPIOB->PUPDR = (GPIOB->PUPDR & ~((3UL << (6U * 2U)) | (3UL << (7U * 2U)))) |
                   (1UL << (6U * 2U)) | (1UL << (7U * 2U));
    GPIOA->PUPDR = (GPIOA->PUPDR & ~(3UL << (8U * 2U))) | (1UL << (8U * 2U));
    GPIOC->PUPDR = (GPIOC->PUPDR & ~(3UL << (9U * 2U))) | (1UL << (9U * 2U));

    motorEnable.write(kMotorEnableInactiveLevel);
    motor1Pwm.period_us(1000);
    motor2Pwm.period_us(1000);
    motor3Pwm.period_us(1000);
    motor4Pwm.period_us(1000);
    ws2811PinTest.period_us(1000);
    ws2811PinTest.write(0.5f);
    setMotorPwmPattern(pwmPhase);

    setupFlowmeterCapture();

    printf("NUCLEO-F446RE Pin-Mapping-Konflikttest gestartet\r\n");
    printf("VCP=PA2/PA3, UART4=PC10/PC11 ALT0, USART6=PC6/PC7\r\n");
    printf("TIM5 bleibt Mbed OS vorbehalten; SWD PA13/PA14 wird nicht umkonfiguriert\r\n");
    printf("I2C sendet Adressbyte 0x40 ohne Daten; NACK ohne Geraet ist normal\r\n");
    fflush(stdout);

    unsigned int ticks = 0;
    while (true) {
        readAndReportUart(raspberryPiUart, "UART4/Raspi");
        readAndReportUart(displayUart, "USART6/Display");

        if ((ticks % 100U) == 0U) { // ca. einmal pro Sekunde
            directionPhase = !directionPhase;
            motor1Dir.write(directionPhase);
            motor2Dir.write(!directionPhase);
            motor3Dir.write(directionPhase);
            motor4Dir.write(!directionPhase);

            sendUartTestMessages();
            const int i2c1Ack = probeI2cAddress(flowmeterI2c);
            const int i2c3Ack = probeI2cAddress(displayI2c);
            reportState(i2c1Ack, i2c3Ack);
        }

        if ((ticks % 500U) == 0U) { // PWM-Duty alle ca. 5 Sekunden wechseln
            setMotorPwmPattern(++pwmPhase);
        }

        ++ticks;
        ThisThread::sleep_for(std::chrono::milliseconds(10));
    }
}
