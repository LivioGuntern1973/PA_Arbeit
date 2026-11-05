#include "mbed.h"
#include "MilkMatePinMap.h"
#include "hal/us_ticker_api.h"

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>

#if !defined(TARGET_NUCLEO_F446RE)
#error "This test is specific to the NUCLEO-F446RE."
#endif
#if !DEVICE_I2CSLAVE
#error "The pin test needs Mbed I2CSlave support."
#endif

using namespace milk_pins;

namespace {
constexpr int MOTOR_PERIOD_US = 1000;
constexpr int LED_PERIOD_US = 500;
constexpr int I2C_ADDRESS = 0x32 << 1; // Mbed takes the shifted 8-bit address.
constexpr int FRAME_SIZE = 16;
constexpr unsigned SPI_BYTES = 64;

// Additional pins used only as test receivers. See src/pinmap_test.md.
constexpr PinName GPIO_FEEDBACK[] = {PB_0, PB_1, PB_4, PB_5};
constexpr PinName SCK_FEEDBACK = PB_8;
constexpr PinName LED_FEEDBACK = PA_10;

struct PinConfig {
    PinName pin;
    const char *name;
    unsigned mode; // STM32 MODER: 0=input, 1=output, 2=alternate function
    unsigned af;
    unsigned pull; // PUPDR: 0=none, 1=up, 2=down
    bool openDrain;
};

constexpr PinConfig PIN_CONFIG[] = {
    {MOTOR_PWM[0], "M1 PWM PB15", 2, 1, 0, false},
    {MOTOR_PWM[1], "M2 PWM PA9", 2, 1, 0, false},
    {MOTOR_PWM[2], "M3 PWM PB13", 2, 1, 0, false},
    {MOTOR_PWM[3], "M4 PWM PA11", 2, 1, 0, false},
    {MOTOR_ENABLE[0], "M1 EN PB14", 1, 0, 0, false},
    {MOTOR_ENABLE[1], "M2 EN PB10", 1, 0, 0, false},
    {MOTOR_ENABLE[2], "M3 EN PC4", 1, 0, 0, false},
    {MOTOR_ENABLE[3], "M4 EN PC5", 1, 0, 0, false},
    {MOTOR_DIR, "DIR PB9", 1, 0, 0, false},
    {MOTOR_ERROR[0], "ERROR1 PC0", 0, 0, 2, false},
    {MOTOR_ERROR[1], "ERROR2 PC1", 0, 0, 2, false},
    {MOTOR_ERROR[2], "ERROR3 PC2", 0, 0, 2, false},
    {MOTOR_ERROR[3], "ERROR4 PC3", 0, 0, 2, false},
    {FLOW_SDA, "I2C1 SDA PB7", 2, 4, 0, true},
    {FLOW_SCL, "I2C1 SCL PB6", 2, 4, 0, true},
    {DISPLAY_SDA, "I2C3 SDA PC9", 2, 4, 0, true},
    {DISPLAY_SCL, "I2C3 SCL PA8", 2, 4, 0, true},
    {DISPLAY_MOSI, "SPI3 MOSI PC12", 2, 6, 2, false},
    {DISPLAY_MISO, "SPI3 MISO PC11", 2, 6, 0, false},
    {DISPLAY_SCK, "SPI3 SCK PC10", 2, 6, 2, false}, // Mode 0: Mbed selects PullDown.
    {DISPLAY_CS, "CS PC7", 1, 0, 0, false},
    {DISPLAY_DC, "DC PC6", 1, 0, 0, false},
    {DISPLAY_RESET, "RESET PB2", 1, 0, 0, false},
    {RASPI_TX, "UART4 TX PA0", 2, 8, 1, false},
    {RASPI_RX, "UART4 RX PA1", 2, 8, 1, false},
    {LED_DATA, "LED PWM PC8", 2, 2, 0, false},
    {milk_pins::CONSOLE_TX, "CONSOLE TX PA2", 2, 7, 1, false},
    {milk_pins::CONSOLE_RX, "CONSOLE RX PA3", 2, 7, 1, false},
    {FLOW_PULSE, "FLOW TIM2 PA15", 2, 1, 2, false},
    {BUTTON[0], "BUTTON1 PA4", 0, 0, 1, false},
    {BUTTON[1], "BUTTON2 PA6", 0, 0, 1, false},
    {BUTTON[2], "BUTTON3 PA7", 0, 0, 1, false},
    {BUTTON[3], "BUTTON4 PB12", 0, 0, 1, false},
    {SWDIO, "SWDIO PA13", 2, 0, 1, false},
    {SWCLK, "SWCLK PA14", 2, 0, 2, false},
    {USER_BUTTON, "USER PC13", 0, 0, 0, false},
    {USER_LED, "LED PA5", 1, 0, 0, false},
    {GPIO_FEEDBACK[0], "TEST PB0", 0, 0, 2, false},
    {GPIO_FEEDBACK[1], "TEST PB1", 0, 0, 2, false},
    {GPIO_FEEDBACK[2], "TEST PB4", 0, 0, 2, false},
    {GPIO_FEEDBACK[3], "TEST PB5", 0, 0, 2, false},
    {SCK_FEEDBACK, "TEST SCK PB8", 0, 0, 2, false},
    {LED_FEEDBACK, "TEST LED PA10", 0, 0, 2, false},
};

constexpr bool uniquePins()
{
    for (unsigned i = 0; i < sizeof(PIN_CONFIG) / sizeof(PIN_CONFIG[0]); ++i) {
        for (unsigned j = i + 1; j < sizeof(PIN_CONFIG) / sizeof(PIN_CONFIG[0]); ++j) {
            if ((unsigned(PIN_CONFIG[i].pin) & 0xFF) == (unsigned(PIN_CONFIG[j].pin) & 0xFF)) {
                return false;
            }
        }
    }
    return true;
}
static_assert(uniquePins(), "Two functions use the same physical pin.");

struct PwmSample {
    uint32_t periods;
    uint32_t periodSum;
    uint32_t pulses;
    uint32_t highSum;
};

class PwmCapture {
public:
    explicit PwmCapture(PinName pin) : input(pin, PullDown)
    {
        input.rise(callback(this, &PwmCapture::rise));
        input.fall(callback(this, &PwmCapture::fall));
    }

    void reset()
    {
        CriticalSectionLock lock;
        lastRise = 0;
        haveRise = false;
        waitingForFall = false;
        periods = periodSum = pulses = highSum = 0;
    }

    PwmSample sample()
    {
        CriticalSectionLock lock;
        return {periods, periodSum, pulses, highSum};
    }

private:
    void rise()
    {
        const uint32_t now = us_ticker_read();
        if (haveRise) {
            periodSum += now - lastRise;
            ++periods;
        }
        lastRise = now;
        haveRise = true;
        waitingForFall = true;
    }

    void fall()
    {
        if (haveRise && waitingForFall) {
            highSum += us_ticker_read() - lastRise;
            ++pulses;
            waitingForFall = false;
        }
    }

    InterruptIn input;
    volatile uint32_t lastRise = 0;
    volatile uint32_t periods = 0;
    volatile uint32_t periodSum = 0;
    volatile uint32_t pulses = 0;
    volatile uint32_t highSum = 0;
    volatile bool haveRise = false;
    volatile bool waitingForFall = false;
};

class TestI2cSlave : public I2CSlave {
public:
    using I2CSlave::I2CSlave;

    bool addressAwaitingBuffer() const
    {
#if DEVICE_I2C_ASYNCH
        const auto &state = _i2c.i2c;
#else
        const auto &state = _i2c;
#endif
        // A pending address while still LISTEN means the application has not
        // armed its RX/TX buffer yet. BUSY_*_LISTEN must keep its interrupts.
        return state.handle.State == HAL_I2C_STATE_LISTEN &&
               (state.pending_slave_rx_maxter_tx || state.pending_slave_tx_master_rx);
    }
};

class TestHardware {
public:
    PwmOut pwm1{MOTOR_PWM[0]}, pwm2{MOTOR_PWM[1]};
    PwmOut pwm3{MOTOR_PWM[2]}, pwm4{MOTOR_PWM[3]};
    PwmOut ledData{LED_DATA};
    DigitalOut en1{MOTOR_ENABLE[0], 0}, en2{MOTOR_ENABLE[1], 0};
    DigitalOut en3{MOTOR_ENABLE[2], 0}, en4{MOTOR_ENABLE[3], 0};
    DigitalOut direction{MOTOR_DIR, 0};
    DigitalOut cs{DISPLAY_CS, 1}, dc{DISPLAY_DC, 0}, reset{DISPLAY_RESET, 0};
    DigitalIn button1{BUTTON[0], PullUp}, button2{BUTTON[1], PullUp};
    DigitalIn button3{BUTTON[2], PullUp}, button4{BUTTON[3], PullUp};
    DigitalIn dirIn{GPIO_FEEDBACK[0], PullDown}, csIn{GPIO_FEEDBACK[1], PullDown};
    DigitalIn dcIn{GPIO_FEEDBACK[2], PullDown}, resetIn{GPIO_FEEDBACK[3], PullDown};
    PwmCapture error1{MOTOR_ERROR[0]}, error2{MOTOR_ERROR[1]};
    PwmCapture error3{MOTOR_ERROR[2]}, error4{MOTOR_ERROR[3]};
    PwmCapture ledIn{LED_FEEDBACK};
    InterruptIn sckIn{SCK_FEEDBACK, PullDown};
    BufferedSerial raspi{RASPI_TX, RASPI_RX, 115200};
    SPI display{DISPLAY_MOSI, DISPLAY_MISO, DISPLAY_SCK};
    I2C flow{FLOW_SDA, FLOW_SCL};
    TestI2cSlave displayI2c{DISPLAY_SDA, DISPLAY_SCL};
    DigitalIn userButton{USER_BUTTON, PullNone};
    DigitalOut userLed{USER_LED, 0};

    PwmOut *motorPwm[4] = {&pwm1, &pwm2, &pwm3, &pwm4};
    PwmCapture *capture[5] = {&error1, &error2, &error3, &error4, &ledIn};
    DigitalOut *gpioOut[8] = {&en1, &en2, &en3, &en4, &direction, &cs, &dc, &reset};
    DigitalIn *gpioIn[8] = {&button1, &button2, &button3, &button4,
                            &dirIn, &csIn, &dcIn, &resetIn};
    volatile uint32_t sckEdges = 0;

    TestHardware()
    {
        // Update every object's cached period before writing any final duty.
        for (auto pwm : motorPwm) {
            pwm->period_us(MOTOR_PERIOD_US);
        }
        ledData.period_us(LED_PERIOD_US);
        ledData.write(0.4f);
        sckIn.rise(callback(this, &TestHardware::clockRise));
        raspi.set_blocking(false);
        display.format(8, 0);
        display.frequency(100000);
        flow.frequency(100000);
        displayI2c.frequency(100000);
        displayI2c.address(I2C_ADDRESS);
    }

private:
    void clockRise() { ++sckEdges; }
};

// TIM2 counts external rising edges on PA15/TI1 in hardware, not GPIO IRQs.
void initFlowCounter()
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM2_FORCE_RESET();
    __HAL_RCC_TIM2_RELEASE_RESET();
    GPIO_InitTypeDef gpio = {};
    gpio.Pin = GPIO_PIN_15;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &gpio);
    TIM2->PSC = 0;
    TIM2->ARR = 0xFFFFFFFFu;
    TIM2->CCMR1 = TIM_CCMR1_CC1S_0; // IC1 mapped to TI1, no input prescaler/filter
    TIM2->CCER = TIM_CCER_CC1E;       // rising polarity
    // TS=101: TI1FP1; SMS=111: external clock mode 1 (RM0390).
    TIM2->SMCR = TIM_SMCR_TS_2 | TIM_SMCR_TS_0 |
                 TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0;
    TIM2->CNT = 0;
    TIM2->CR1 = TIM_CR1_CEN;
}

bool checkPinConfiguration()
{
    bool ok = true;
    for (const auto &pin : PIN_CONFIG) {
        const unsigned portIndex = (static_cast<unsigned>(pin.pin) >> 4) & 0xF;
        GPIO_TypeDef *port = portIndex == 0 ? GPIOA : portIndex == 1 ? GPIOB : GPIOC;
        const unsigned number = static_cast<unsigned>(pin.pin) & 0xF;
        const unsigned mode = (port->MODER >> (number * 2)) & 3;
        const unsigned pull = (port->PUPDR >> (number * 2)) & 3;
        const unsigned af = (port->AFR[number / 8] >> ((number % 8) * 4)) & 0xF;
        const bool od = (port->OTYPER & (1u << number)) != 0;
        if (mode != pin.mode || pull != pin.pull ||
            (mode == 2 && (af != pin.af || od != pin.openDrain))) {
            printf("  PIN FAIL %-18s mode=%u/%u AF=%u/%u pull=%u/%u OD=%u/%u\r\n",
                   pin.name, mode, pin.mode, af, pin.af, pull, pin.pull,
                   unsigned(od), unsigned(pin.openDrain));
            ok = false;
        }
    }
    return ok;
}

std::atomic<uint32_t> slaveWrites{0}, slaveReads{0}, slaveErrors{0};
std::atomic<uint32_t> slavePolls{0}, slaveWriteRequests{0}, slaveReadRequests{0};
std::atomic<uint32_t> slaveAddressDeferrals{0};

using IrqHandler = void (*)();
IrqHandler originalI2c3EventHandler = nullptr;
TestI2cSlave *guardedI2cSlave = nullptr;

void guardedI2c3EventHandler()
{
    // Let Mbed/HAL identify the address and set its pending-slave flag first.
    originalI2c3EventHandler();
    if (guardedI2cSlave->addressAwaitingBuffer()) {
        // STM32F4 keeps ADDR asserted until a buffer is armed. Without this
        // deferral, the event IRQ can retrigger continuously and prevent the
        // application thread from ever reaching I2CSlave::read()/write().
        // Keep ADDR and clock stretching intact; only pause event/buffer IRQs.
        // HAL_I2C_Slave_Seq_*_IT() clears ADDR and re-enables these IRQs when
        // the slave thread arms the buffer. Data-transfer IRQs stay enabled.
        I2C3->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN);
        slaveAddressDeferrals.fetch_add(1, std::memory_order_relaxed);
    }
}

void installI2c3EventGuard(TestI2cSlave &slave)
{
    const auto guardedVector = reinterpret_cast<uint32_t>(&guardedI2c3EventHandler);
    if (NVIC_GetVector(I2C3_EV_IRQn) != guardedVector) {
        CriticalSectionLock lock;
        guardedI2cSlave = &slave;
        originalI2c3EventHandler = reinterpret_cast<IrqHandler>(NVIC_GetVector(I2C3_EV_IRQn));
        NVIC_SetVector(I2C3_EV_IRQn, guardedVector);
    }
}

void printI2cDiagnostics()
{
    // Read GPIO input data without creating DigitalIn objects: those would
    // replace the I2C alternate functions. CR1/CR2/OAR1 have no read-to-clear
    // flags; deliberately do not read the SR1/SR2 sequence during slave IRQs.
    printf("  I2C idle SDA PB7/PC9=%u/%u SCL PB6/PA8=%u/%u (expected all 1)\r\n",
           unsigned((GPIOB->IDR >> 7) & 1), unsigned((GPIOC->IDR >> 9) & 1),
           unsigned((GPIOB->IDR >> 6) & 1), unsigned((GPIOA->IDR >> 8) & 1));
    printf("  I2C3 listen PE=%u ACK=%u eventIRQ=%u OAR1=0x%04lX\r\n",
           unsigned(bool(I2C3->CR1 & I2C_CR1_PE)),
           unsigned(bool(I2C3->CR1 & I2C_CR1_ACK)),
           unsigned(bool(I2C3->CR2 & I2C_CR2_ITEVTEN)), (unsigned long)I2C3->OAR1);
}

void serveI2c(TestHardware *hw)
{
    // Must run in a second thread: a synchronous master call waits for the slave.
    // The master driver uses IRQs; its polling loop alone cannot service I2CSlave.
    char received[FRAME_SIZE] = {};
    char response[FRAME_SIZE] = {};
    while (true) {
        // Mbed may restore its vector during recovery; wrap it again before
        // arming another buffer. No global framework files are modified.
        installI2c3EventGuard(hw->displayI2c);
        ++slavePolls;
        switch (hw->displayI2c.receive()) {
            case I2CSlave::WriteAddressed:
                ++slaveWriteRequests;
                if (hw->displayI2c.read(received, FRAME_SIZE) == 0) {
                    for (int i = 0; i < FRAME_SIZE; ++i) {
                        response[i] = static_cast<char>(received[i] ^ 0xA5);
                    }
                    ++slaveWrites;
                } else {
                    ++slaveErrors;
                }
                break;
            case I2CSlave::ReadAddressed:
                ++slaveReadRequests;
                if (hw->displayI2c.write(response, FRAME_SIZE) == 0) {
                    ++slaveReads;
                } else {
                    ++slaveErrors;
                }
                break;
            default:
                break;
        }
        // Yield after errors as well so a stale pending flag cannot starve
        // the master thread and its timeout/reporting path.
        ThisThread::sleep_for(1ms);
    }
}

bool testI2c(TestHardware &hw, unsigned round)
{
    const auto errorsBefore = slaveErrors.load();
    const auto writesBefore = slaveWrites.load();
    const auto readsBefore = slaveReads.load();
    const auto pollsBefore = slavePolls.load();
    const auto writeRequestsBefore = slaveWriteRequests.load();
    const auto readRequestsBefore = slaveReadRequests.load();
    const auto deferralsBefore = slaveAddressDeferrals.load();
    printI2cDiagnostics();
    unsigned good = 0;
    for (unsigned frame = 0; frame < 4; ++frame) {
        char sent[FRAME_SIZE], expected[FRAME_SIZE], received[FRAME_SIZE] = {};
        for (int i = 0; i < FRAME_SIZE; ++i) {
            sent[i] = static_cast<char>(round * 13 + frame * 47 + i * 17);
            expected[i] = static_cast<char>(sent[i] ^ 0xA5);
        }
        printf("  I2C frame %u: writing...\r\n", frame);
        const int tx = hw.flow.write(I2C_ADDRESS, sent, FRAME_SIZE);
        ThisThread::sleep_for(2ms); // Let the slave finish preparing its reply.
        printf("  I2C frame %u: tx=%d, reading...\r\n", frame, tx);
        const int rx = hw.flow.read(I2C_ADDRESS, received, FRAME_SIZE);
        ThisThread::sleep_for(2ms);
        if (tx == 0 && rx == 0 && memcmp(received, expected, FRAME_SIZE) == 0) {
            ++good;
        } else {
            printf("  I2C frame %u FAIL tx=%d rx=%d\r\n", frame, tx, rx);
        }
    }
    const auto writes = slaveWrites.load() - writesBefore;
    const auto reads = slaveReads.load() - readsBefore;
    printf("  I2C1 <-> I2C3: %u/4 frames, slave RX/TX=%lu/%lu\r\n",
           good, (unsigned long)writes, (unsigned long)reads);
    printf("  I2C slave polls=%lu addressed RX/TX=%lu/%lu errors=%lu\r\n",
           (unsigned long)(slavePolls.load() - pollsBefore),
           (unsigned long)(slaveWriteRequests.load() - writeRequestsBefore),
           (unsigned long)(slaveReadRequests.load() - readRequestsBefore),
           (unsigned long)(slaveErrors.load() - errorsBefore));
    printf("  I2C address-to-thread deferrals=%lu\r\n",
           (unsigned long)(slaveAddressDeferrals.load() - deferralsBefore));
    return good == 4 && writes == 4 && reads == 4 && slaveErrors.load() == errorsBefore;
}

bool testUart(TestHardware &hw, unsigned round)
{
    char discarded;
    while (hw.raspi.read(&discarded, 1) == 1) {}
    unsigned good = 0;
    // One byte at a time prevents a loopback RX-buffer overrun.
    for (unsigned i = 0; i < 64; ++i) {
        const char sent = static_cast<char>(i * 37 + round);
        char received = 0;
        Timer deadline;
        deadline.start();
        bool sentOk = false, receivedOk = false;
        while (deadline.elapsed_time() < 20ms) {
            if (!sentOk) {
                sentOk = hw.raspi.write(&sent, 1) == 1;
            }
            if (hw.raspi.read(&received, 1) == 1) {
                receivedOk = true;
                break;
            }
            ThisThread::sleep_for(1ms);
        }
        if (sentOk && receivedOk && received == sent) {
            ++good;
        }
    }
    printf("  UART4 TX -> RX: %u/64 bytes (115200 8N1)\r\n", good);
    return good == 64;
}

bool testSpi(TestHardware &hw, unsigned round)
{
    unsigned good = 0;
    const uint32_t before = hw.sckEdges;
    hw.cs = 0;
    for (unsigned i = 0; i < SPI_BYTES; ++i) {
        const int sent = (i * 53 + round) & 0xFF;
        if (hw.display.write(sent) == sent) {
            ++good;
        }
    }
    hw.cs = 1;
    wait_us(100); // Allow the final SCK interrupt to finish.
    const uint32_t clocks = hw.sckEdges - before;
    printf("  SPI3 MOSI -> MISO: %u/%u bytes, external SCK=%lu/%u\r\n",
           good, SPI_BYTES, (unsigned long)clocks, SPI_BYTES * 8);
    return good == SPI_BYTES && clocks == SPI_BYTES * 8;
}

bool testGpio(TestHardware &hw)
{
    bool ok = true;
    // All-zero/all-one, walking-one/walking-zero: also catches swapped loops
    // and shorts between the eight digital nets.
    for (unsigned patternIndex = 0; patternIndex < 20; ++patternIndex) {
        const unsigned pattern = patternIndex == 0 ? 0 : patternIndex == 1 ? 0xFF :
            patternIndex < 10 ? (1u << (patternIndex - 2)) :
            patternIndex < 18 ? (0xFF ^ (1u << (patternIndex - 10))) :
            patternIndex == 18 ? 0x55 : 0xAA;
        for (unsigned i = 0; i < 8; ++i) {
            *hw.gpioOut[i] = (pattern >> i) & 1;
        }
        ThisThread::sleep_for(2ms);
        unsigned actual = 0;
        for (unsigned i = 0; i < 8; ++i) {
            actual |= unsigned(hw.gpioIn[i]->read()) << i;
        }
        if (actual != pattern) {
            if (ok) {
                printf("  GPIO FAIL expected=0x%02X received=0x%02X xor=0x%02X\r\n",
                       pattern, actual, pattern ^ actual);
            }
            ok = false;
        }
    }
    for (auto output : hw.gpioOut) { *output = 0; }
    hw.cs = 1;
    return ok;
}

void setMotorDuties(TestHardware &hw, const unsigned duty[4])
{
    for (unsigned i = 0; i < 4; ++i) {
        // RM0390 16.3.11: when only OCxN is enabled (CCxE=0, CCxNE=1),
        // it follows OCxREF without complementation. Mbed uses that mode on
        // PB13/PB15 here, with CCxNP=0. No extra software inversion is needed.
        hw.motorPwm[i]->write(duty[i] / 100.0f);
    }
}

bool checkTim1Outputs()
{
    const uint32_t mask = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC1P | TIM_CCER_CC1NP |
                          TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC2P | TIM_CCER_CC2NP |
                          TIM_CCER_CC3E | TIM_CCER_CC3NE | TIM_CCER_CC3P | TIM_CCER_CC3NP |
                          TIM_CCER_CC4E | TIM_CCER_CC4P;
    const uint32_t expected = TIM_CCER_CC1NE | TIM_CCER_CC2E |
                              TIM_CCER_CC3NE | TIM_CCER_CC4E;
    const bool ok = (TIM1->CCER & mask) == expected;
    if (!ok) {
        printf("  TIM1 CCER=0x%04lX, expected enables/polarities=0x%04lX\r\n",
               (unsigned long)TIM1->CCER, (unsigned long)expected);
    }
    return ok;
}

unsigned samplePa15Edges()
{
    // Keep PA15 in AF1. IDR also exposes its electrical input in AF mode.
    unsigned rises = 0;
    bool previous = (GPIOA->IDR & GPIO_PIN_15) != 0;
    const uint32_t start = us_ticker_read();
    while (us_ticker_read() - start < 10000) {
        const bool level = (GPIOA->IDR & GPIO_PIN_15) != 0;
        if (level && !previous) { ++rises; }
        previous = level;
        wait_us(5);
    }
    return rises;
}

bool checkPwm(PwmCapture &capture, const char *name, unsigned period, unsigned duty)
{
    const auto s = capture.sample();
    const uint32_t measuredPeriod = s.periods ? s.periodSum / s.periods : 0;
    const uint32_t measuredHigh = s.pulses ? s.highSum / s.pulses : 0;
    const uint32_t measuredDuty = measuredPeriod ? measuredHigh * 100 / measuredPeriod : 0;
    // Interrupt latency during simultaneous bus traffic: 5% period, 8 percentage
    // points duty tolerance. This is a pin/function test, not timing calibration.
    const bool ok = s.periods >= 50 && s.pulses >= 50 &&
        measuredPeriod >= period * 95 / 100 && measuredPeriod <= period * 105 / 100 &&
        measuredDuty + 8 >= duty && measuredDuty <= duty + 8;
    printf("  %-12s %s T=%lu us HIGH=%lu us duty=%lu%% (expected %u%%)\r\n",
           name, ok ? "PASS" : "FAIL", (unsigned long)measuredPeriod,
           (unsigned long)measuredHigh, (unsigned long)measuredDuty, duty);
    return ok;
}

void printResult(const char *name, bool ok, unsigned &failures)
{
    printf("[%s] %s\r\n", ok ? "PASS" : "FAIL", name);
    if (!ok) { ++failures; }
}

} // namespace

int main()
{
    // Let OpenOCD finish its reset/disconnect sequence before sending UART
    // bytes to the ST-Link/V2-1 bridge (older probe firmware is sensitive to
    // UART traffic while its debug USB interface is being reconfigured).
    ThisThread::sleep_for(3s);
    setvbuf(stdout, nullptr, _IONBF, 0);
    printf("\r\nMilkMate NUCLEO-F446RE pin test v4\r\n");
    printf("Only a bare Nucleo with test jumpers! Wiring: src/pinmap_test.md\r\n");
    printf("Initializing ALL mapped peripherals simultaneously...\r\n");
    static TestHardware hw;
    initFlowCounter();
    installI2c3EventGuard(hw.displayI2c);
    printf("I2C3 address-to-thread interrupt guard installed.\r\n");
    static Thread slaveThread(osPriorityAboveNormal, 4096, nullptr, "i2c3-slave");
    const osStatus slaveStart = slaveThread.start(callback(serveI2c, &hw));
    printf("I2C slave thread start=%d (expected 0)\r\n", int(slaveStart));
    FileHandle *console = mbed_file_handle(STDIN_FILENO);
    // Mbed's default DirectSerial does not implement nonblocking read().
    // Poll first and read exactly one byte so the automatic test never waits
    // for console input (works with BufferedSerial consoles as well).
    printf("Console RX ready. Type any character to test the return path.\r\n");
    printf("Press/release blue B1; observe green LD2 blinking.\r\n");
    ThisThread::sleep_for(2s);

    unsigned round = 0, totalFailures = 0, buttonChanges = 0;
    bool consoleSeen = false;
    int buttonState = hw.userButton.read();
    while (true) {
        ++round;
        unsigned failures = 0;
        printf("\r\n--- Round %u ---\r\n", round);
        const unsigned dutiesA[] = {20, 35, 60, 80};
        const unsigned dutiesB[] = {75, 55, 40, 25};
        const unsigned *duties = (round & 1) ? dutiesA : dutiesB;
        setMotorDuties(hw, duties);
        ThisThread::sleep_for(10ms);
        for (auto capture : hw.capture) { capture->reset(); }
        Timer window;
        window.start();
        const uint32_t countBefore = TIM2->CNT;
        const unsigned pa15Edges = samplePa15Edges();
        printResult("Pin modes / AF / pull resistors", checkPinConfiguration(), failures);
        printResult("GPIO enables, DIR, display controls, button inputs", testGpio(hw), failures);
        printResult("UART4 loop", testUart(hw, round), failures);
        printResult("SPI3 loop + physical SCK", testSpi(hw, round), failures);
        printResult("I2C1 master / I2C3 slave, both data directions", testI2c(hw, round), failures);
        while (window.elapsed_time() < 500ms) { ThisThread::sleep_for(10ms); }
        const uint32_t countAfter = TIM2->CNT;
        const uint32_t elapsedUs = std::chrono::duration_cast<std::chrono::microseconds>(
            window.elapsed_time()).count();
        const uint32_t counts = countAfter - countBefore;
        const uint32_t expectedCounts = elapsedUs / LED_PERIOD_US;
        const uint32_t tolerance = expectedCounts / 50 + 3;
        printf("  TIM2 PA15: %lu external pulses, expected about %lu\r\n",
               (unsigned long)counts, (unsigned long)expectedCounts);
        printf("  PA15 electrical input: %u rising edges / 10 ms (expected about 20)\r\n",
               pa15Edges);
        if (counts == 0) {
            printf("  TIM2 CR1=0x%lX SMCR=0x%lX CCMR1=0x%lX CCER=0x%lX clock=%u\r\n",
                   (unsigned long)TIM2->CR1, (unsigned long)TIM2->SMCR,
                   (unsigned long)TIM2->CCMR1, (unsigned long)TIM2->CCER,
                   unsigned(bool(RCC->APB1ENR & RCC_APB1ENR_TIM2EN)));
        }
        printResult("PC8 -> PA15 hardware pulse counter",
                    counts + tolerance >= expectedCounts && counts <= expectedCounts + tolerance,
                    failures);
        bool pwmOk = checkTim1Outputs();
        const char *pwmNames[] = {"M1 PB15", "M2 PA9", "M3 PB13", "M4 PA11"};
        for (unsigned i = 0; i < 4; ++i) {
            pwmOk = checkPwm(*hw.capture[i], pwmNames[i], MOTOR_PERIOD_US, duties[i]) && pwmOk;
        }
        printResult("Four TIM1 PWM channels -> PC0..PC3", pwmOk, failures);
        printResult("PC8 TIM3 PWM -> PA10", checkPwm(hw.ledIn, "LED PC8", LED_PERIOD_US, 40), failures);
        printResult("Pin configuration after bus traffic", checkPinConfiguration(), failures);
        totalFailures += failures;
        printf("AUTO %s: %u failed groups; total failed groups=%u\r\n",
               failures ? "FAIL" : "PASS", failures, totalFailures);

        for (unsigned tick = 0; tick < 50; ++tick) {
            hw.userLed = (tick / 10) & 1;
            const int state = hw.userButton.read();
            if (state != buttonState) {
                ThisThread::sleep_for(20ms);
                if (hw.userButton.read() == state) {
                    buttonState = state;
                    ++buttonChanges;
                    printf("[MANUAL] B1 PC13=%d, stable changes=%u\r\n", state, buttonChanges);
                }
            }
            char c;
            if ((console->poll(POLLIN) & POLLIN) && console->read(&c, 1) == 1) {
                consoleSeen = true;
                printf("[MANUAL] Console RX received 0x%02X\r\n", unsigned(uint8_t(c)));
            }
            ThisThread::sleep_for(20ms);
        }
        hw.userLed = 0;
        printf("MANUAL: B1 press/release=%s, console RX=%s; LD2 requires visual check.\r\n",
               buttonChanges >= 2 ? "seen" : "pending", consoleSeen ? "seen" : "pending");
    }
}
