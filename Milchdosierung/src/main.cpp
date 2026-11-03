#include "stm32f4xx_hal.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define CONSOLE_BAUD 115200U
#define FEEDBACK_PORT GPIOA
#define FEEDBACK_PIN GPIO_PIN_4 /* CN7-32 */

static UART_HandleTypeDef huart2;
static UART_HandleTypeDef huart4;
static SPI_HandleTypeDef hspi3;
static I2C_HandleTypeDef hi2c1;
static I2C_HandleTypeDef hi2c3;
static TIM_HandleTypeDef htim1;
static TIM_HandleTypeDef htim2;
static TIM_HandleTypeDef htim3;
static uint8_t console_ready;
static uint8_t i2c_pair_initialized;
static volatile uint8_t i2c_rx_complete;
static volatile uint8_t i2c_tx_armed;
static volatile uint8_t i2c_tx_complete;
static volatile uint8_t i2c_slave_error;
static I2C_HandleTypeDef * volatile active_i2c_slave;
static uint8_t i2c_rx_buffer[8];
static uint8_t i2c_reply_buffer[8];

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    const char *name;
    const char *cn;
} PinRow;

static const PinRow gpio_outputs[] = {
    {GPIOB, GPIO_PIN_14, "Motor 1 DIR", "CN10-28"},
    {GPIOB, GPIO_PIN_10, "Motor 2 DIR", "CN10-25"},
    {GPIOC, GPIO_PIN_4,  "Motor 3 DIR", "CN10-34"},
    {GPIOC, GPIO_PIN_5,  "Motor 4 DIR", "CN10-6"},
    {GPIOB, GPIO_PIN_9,  "Motoren Enable", "CN10-5"},
    {GPIOC, GPIO_PIN_7,  "Display CS (optional)", "CN10-19"},
    {GPIOC, GPIO_PIN_6,  "Display D/C (optional)", "CN10-4"},
    {GPIOB, GPIO_PIN_2,  "Display RESET (optional)", "CN10-22"},
};

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    const char *name;
    const char *cn;
    uint32_t pull;
} InputRow;

static const InputRow gpio_inputs[] = {
    {GPIOC, GPIO_PIN_0, "Motor 1 Fehler", "CN7-38", GPIO_PULLDOWN},
    {GPIOC, GPIO_PIN_1, "Motor 2 Fehler", "CN7-36", GPIO_PULLDOWN},
    {GPIOC, GPIO_PIN_2, "Motor 3 Fehler", "CN7-35", GPIO_PULLDOWN},
    {GPIOC, GPIO_PIN_3, "Motor 4 Fehler", "CN7-37", GPIO_PULLDOWN},
    {GPIOA, GPIO_PIN_4, "Taster 1", "CN7-32", GPIO_PULLUP},
    {GPIOA, GPIO_PIN_6, "Taster 2", "CN10-13", GPIO_PULLUP},
    {GPIOA, GPIO_PIN_7, "Taster 3", "CN10-15", GPIO_PULLUP},
    {GPIOB, GPIO_PIN_12,"Taster 4", "CN10-16", GPIO_PULLUP},
    {GPIOC, GPIO_PIN_13,"User Button", "CN7-23", GPIO_PULLUP},
};

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    const char *name;
    const char *cn;
    uint32_t channel;
    uint8_t complementary;
    uint8_t timer3;
} PwmRow;

static const PwmRow pwm_outputs[] = {
    {GPIOB, GPIO_PIN_15, "Motor 1 PWM (TIM1_CH3N)", "CN10-26", TIM_CHANNEL_3, 1U, 0U},
    {GPIOA, GPIO_PIN_9,  "Motor 2 PWM (TIM1_CH2)",  "CN10-21", TIM_CHANNEL_2, 0U, 0U},
    {GPIOB, GPIO_PIN_13, "Motor 3 PWM (TIM1_CH1N)", "CN10-30", TIM_CHANNEL_1, 1U, 0U},
    {GPIOA, GPIO_PIN_11, "Motor 4 PWM (TIM1_CH4)",  "CN10-14", TIM_CHANNEL_4, 0U, 0U},
    {GPIOC, GPIO_PIN_8,  "Neopixel data test (TIM3_CH3)", "CN10-2", TIM_CHANNEL_3, 0U, 1U},
};

static void SystemClock_Config(void);
static void Error_Handler(void);

static void port_clock_enable(GPIO_TypeDef *port)
{
    if (port == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
    else if (port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
    else if (port == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
}

static void gpio_config_input(GPIO_TypeDef *port, uint16_t pin, uint32_t pull)
{
    GPIO_InitTypeDef init = {0};
    port_clock_enable(port);
    init.Pin = pin;
    init.Mode = GPIO_MODE_INPUT;
    init.Pull = pull;
    init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(port, &init);
}

static void gpio_config_output(GPIO_TypeDef *port, uint16_t pin)
{
    GPIO_InitTypeDef init = {0};
    port_clock_enable(port);
    init.Pin = pin;
    init.Mode = GPIO_MODE_OUTPUT_PP;
    init.Pull = GPIO_NOPULL;
    init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(port, &init);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

static void gpio_config_af(GPIO_TypeDef *port, uint16_t pin, uint32_t af)
{
    GPIO_InitTypeDef init = {0};
    port_clock_enable(port);
    init.Pin = pin;
    init.Mode = GPIO_MODE_AF_PP;
    init.Pull = GPIO_NOPULL;
    init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    init.Alternate = af;
    HAL_GPIO_Init(port, &init);
}

static void console_write(const char *s)
{
    if (console_ready) {
        HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)strlen(s), HAL_MAX_DELAY);
    }
}

static void console_printf(const char *format, ...)
{
    char buffer[320];
    va_list args;
    va_start(args, format);
    int n = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    if (n > 0) {
        size_t length = (size_t)n;
        if (length >= sizeof(buffer)) length = sizeof(buffer) - 1U;
        if (console_ready) HAL_UART_Transmit(&huart2, (uint8_t *)buffer, (uint16_t)length, HAL_MAX_DELAY);
    }
}

static void console_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &gpio);

    huart2.Instance = USART2;
    huart2.Init.BaudRate = CONSOLE_BAUD;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
    console_ready = 1U;
}

static void console_read_line(char *buffer, size_t capacity)
{
    size_t used = 0U;
    for (;;) {
        uint8_t ch;
        if (HAL_UART_Receive(&huart2, &ch, 1U, HAL_MAX_DELAY) != HAL_OK) continue;
        if (ch == '\r') continue;
        if (ch == '\n') {
            buffer[used] = '\0';
            console_write("\r\n");
            return;
        }
        if (used + 1U < capacity) buffer[used++] = (char)ch;
    }
}

static uint8_t wait_go(const char *instruction)
{
    char line[64];
    console_printf("\r\n%s\r\nTippe go + Enter (q = abbrechen): ", instruction);
    console_read_line(line, sizeof(line));
    if (strcmp(line, "q") == 0 || strcmp(line, "Q") == 0) {
        console_write("Abgebrochen.\r\n");
        return 0U;
    }
    if (strcmp(line, "go") != 0 && strcmp(line, "GO") != 0) {
        console_write("Erwartet wurde 'go'; Schritt uebersprungen.\r\n");
        return 0U;
    }
    return 1U;
}

static uint8_t status_ok(HAL_StatusTypeDef status, const char *operation)
{
    if (status == HAL_OK) return 1U;
    console_printf("FEHLER: %s (HAL status %u)\r\n", operation, (unsigned)status);
    return 0U;
}

static void print_help(void)
{
    console_write(
        "\r\nBefehle:\r\n"
        "  help    diese Hilfe\r\n"
        "  map     Pin-/CN-Uebersicht\r\n"
        "  gpio    digitale Ausgaenge gegen CN7-32 pruefen\r\n"
        "  spi3    SPI3 MOSI->MISO Loopback\r\n"
        "  uart4   UART4 TX->RX Loopback\r\n"
        "  i2c1    I2C1 Master -> I2C3 Slave Datentest\r\n"
        "  i2c3    I2C3 Master -> I2C1 Slave Datentest\r\n"
        "  pwm     vier TIM1-Ausgaenge + TIM3/PC8 pruefen\r\n"
        "  flow    TIM1 PWM gegen TIM2 Eingang capture pruefen\r\n"
        "  inputs  GPIO-Eingaenge live beobachten (q = Ende)\r\n"
        "  led     User LED PA5 blinken lassen\r\n"
        "  all     GPIO, SPI, UART4, I2C-Leitungen, PWM und Flow nacheinander\r\n\r\n");
}

static void print_map(void)
{
    console_write(
        "\r\nPinmapping / CN-Referenz:\r\n"
        "  TIM1 PWM: PB15 CN10-26 (CH3N), PA9 CN10-21 (CH2), PB13 CN10-30 (CH1N), PA11 CN10-14 (CH4)\r\n"
        "  TIM3 PWM: PC8 CN10-2 (CH3, Testsignal; kein WS2811-Protokoll)\r\n"
        "  DIR/Steuerung: PB14 CN10-28, PB10 CN10-25, PC4 CN10-34, PC5 CN10-6, PB9 CN10-5\r\n"
        "  Display GPIO: PC7 CS CN10-19, PC6 D/C CN10-4, PB2 RESET CN10-22 (optional)\r\n"
        "  I2C1: PB7 SDA CN7-21, PB6 SCL CN10-17; I2C3: PC9 SDA CN10-1, PA8 SCL CN10-23\r\n"
        "  I2C-Loop: SDA1<->SDA3 und SCL1<->SCL3 verbinden; i2c1/i2c3 tauschen Testbytes aus\r\n"
        "  SPI3: PC10 SCK CN7-1, PC11 MISO CN7-2, PC12 MOSI CN7-3\r\n"
        "  UART4: PA1 RX CN7-30, PA0 TX CN7-28\r\n"
        "  Flow: PA15 TIM2_CH1 CN10-11 (Eingang)\r\n"
        "  GPIO-Eingaenge: PC0/1/2/3 CN7-38/36/35/37; PA4 CN7-32; PA6 CN10-13; PA7 CN10-15; PB12 CN10-16; PC13 CN7-23\r\n"
        "  CLI: USART2 PA2/PA3 ueber ST-LINK VCP; nicht fuer Loopback umkonfigurieren\r\n"
        "  GPIO Rueckmelde-Eingang: PA4 CN7-32; User LED PA5; SWD PA13/PA14 bleibt fuer Debug frei.\r\n\r\n");
}

static void test_gpio_outputs(void)
{
    char instruction[180];
    gpio_config_input(FEEDBACK_PORT, FEEDBACK_PIN, GPIO_PULLDOWN);
    console_write("\r\nGPIO-Test: jeweils genau einen Jumper vom genannten Ausgang nach CN7-32 (PA4).\r\n");
    for (size_t i = 0U; i < sizeof(gpio_outputs) / sizeof(gpio_outputs[0]); ++i) {
        const PinRow *row = &gpio_outputs[i];
        gpio_config_output(row->port, row->pin);
        (void)snprintf(instruction, sizeof(instruction), "%s (%s) -> CN7-32 (PA4)", row->name, row->cn);
        if (!wait_go(instruction)) continue;
        HAL_GPIO_WritePin(row->port, row->pin, GPIO_PIN_SET);
        HAL_Delay(3U);
        GPIO_PinState high = HAL_GPIO_ReadPin(FEEDBACK_PORT, FEEDBACK_PIN);
        HAL_GPIO_WritePin(row->port, row->pin, GPIO_PIN_RESET);
        HAL_Delay(3U);
        GPIO_PinState low = HAL_GPIO_ReadPin(FEEDBACK_PORT, FEEDBACK_PIN);
        console_printf("  %s: %s (HIGH=%u, LOW=%u)\r\n", row->name,
                       (high == GPIO_PIN_SET && low == GPIO_PIN_RESET) ? "PASS" : "FAIL",
                       (unsigned)high, (unsigned)low);
    }
}

static void test_spi3(void)
{
    uint8_t tx[] = {0x55U, 0xA3U, 0x00U, 0xFFU, 0x39U, 0xC6U, 0x81U, 0x18U};
    uint8_t rx[sizeof(tx)] = {0};
    console_write("\r\nSPI3: PC12 MOSI CN7-3 koennen Sie mit PC11 MISO CN7-2 verbinden.\r\n");
    gpio_config_af(GPIOC, GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12, GPIO_AF6_SPI3);
    __HAL_RCC_SPI3_CLK_ENABLE();
    hspi3.Instance = SPI3;
    hspi3.Init.Mode = SPI_MODE_MASTER;
    hspi3.Init.Direction = SPI_DIRECTION_2LINES;
    hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi3.Init.NSS = SPI_NSS_SOFT;
    hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi3.Init.CRCPolynomial = 7U;
    if (!status_ok(HAL_SPI_Init(&hspi3), "SPI3 init")) return;
    if (!wait_go("Jumper PC12/MOSI CN7-3 auf PC11/MISO CN7-2 stecken.")) return;
    if (!status_ok(HAL_SPI_TransmitReceive(&hspi3, tx, rx, sizeof(tx), 250U), "SPI3 transfer")) return;
    if (memcmp(tx, rx, sizeof(tx)) == 0) console_write("  SPI3: PASS, alle Bytes kamen identisch zurueck.\r\n");
    else {
        console_write("  SPI3: FAIL, gesendet/empfangen:\r\n  TX: ");
        for (size_t i = 0U; i < sizeof(tx); ++i) console_printf("%02X ", tx[i]);
        console_write("\r\n  RX: ");
        for (size_t i = 0U; i < sizeof(rx); ++i) console_printf("%02X ", rx[i]);
        console_write("\r\n");
    }
}

static void test_uart4(void)
{
    uint8_t tx[] = {0x55U, 0xC3U, 0x00U, 0xFFU, 0x27U, 0xA9U};
    uint8_t rx[sizeof(tx)] = {0};
    console_write("\r\nUART4: TX PA0 CN7-28 nach RX PA1 CN7-30 verbinden.\r\n");
    gpio_config_af(GPIOA, GPIO_PIN_0 | GPIO_PIN_1, GPIO_AF8_UART4);
    __HAL_RCC_UART4_CLK_ENABLE();
    huart4.Instance = UART4;
    huart4.Init.BaudRate = 115200U;
    huart4.Init.WordLength = UART_WORDLENGTH_8B;
    huart4.Init.StopBits = UART_STOPBITS_1;
    huart4.Init.Parity = UART_PARITY_NONE;
    huart4.Init.Mode = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;
    if (!status_ok(HAL_UART_Init(&huart4), "UART4 init")) return;
    if (!wait_go("Jumper PA0/TX CN7-28 auf PA1/RX CN7-30 stecken.")) return;
    if (!status_ok(HAL_UART_Transmit(&huart4, tx, sizeof(tx), 250U), "UART4 senden")) return;
    if (!status_ok(HAL_UART_Receive(&huart4, rx, sizeof(rx), 500U), "UART4 empfangen")) {
        console_write("  Pruefe Jumper, CN-Nummern und gemeinsame Masse.\r\n");
        return;
    }
    console_write(memcmp(tx, rx, sizeof(tx)) == 0 ? "  UART4: PASS, alle Bytes kamen identisch zurueck.\r\n"
                                                   : "  UART4: FAIL, empfangene Daten stimmen nicht.\r\n");
}

static uint8_t i2c_init_pair(void)
{
    GPIO_InitTypeDef gpio = {0};
    if (i2c_pair_initialized) return 1U;
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_I2C3_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_AF_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = GPIO_PIN_8;
    gpio.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOC, &gpio);

    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 10000U;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = (0x42U << 1U);
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0U;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (!status_ok(HAL_I2C_Init(&hi2c1), "I2C1 init")) return 0U;

    hi2c3.Instance = I2C3;
    hi2c3.Init.ClockSpeed = 10000U;
    hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c3.Init.OwnAddress1 = (0x42U << 1U);
    hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c3.Init.OwnAddress2 = 0U;
    hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (!status_ok(HAL_I2C_Init(&hi2c3), "I2C3 init")) return 0U;

    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 5U, 0U);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 5U, 0U);
    HAL_NVIC_SetPriority(I2C3_EV_IRQn, 5U, 0U);
    HAL_NVIC_SetPriority(I2C3_ER_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
    HAL_NVIC_EnableIRQ(I2C3_EV_IRQn);
    HAL_NVIC_EnableIRQ(I2C3_ER_IRQn);
    i2c_pair_initialized = 1U;
    return 1U;
}

static void test_i2c_loop(uint8_t master_is_i2c1)
{
    static const uint8_t tx[] = {0x55U, 0xC3U, 0x00U, 0xFFU, 0x27U, 0xA9U, 0x81U, 0x18U};
    uint8_t rx[sizeof(tx)] = {0};
    I2C_HandleTypeDef *master = master_is_i2c1 ? &hi2c1 : &hi2c3;
    I2C_HandleTypeDef *slave = master_is_i2c1 ? &hi2c3 : &hi2c1;
    const char *master_name = master_is_i2c1 ? "I2C1" : "I2C3";
    const char *slave_name = master_is_i2c1 ? "I2C3" : "I2C1";

    console_printf("\r\n%s Master -> %s Slave: beide I2C-Busse parallel verbinden.\r\n", master_name, slave_name);
    console_write("  SDA: PB7 CN7-21 <-> PC9 CN10-1\r\n  SCL: PB6 CN10-17 <-> PA8 CN10-23\r\n");
    console_write("I2C1 und I2C3 verwenden interne Pull-ups und 10 kHz; alle externen I2C-Geraete abziehen.\r\n");
    if (!wait_go("Zwei Jumper stecken: SDA1<->SDA3 und SCL1<->SCL3.")) return;
    if (!i2c_init_pair()) return;
    if (HAL_I2C_GetState(slave) != HAL_I2C_STATE_READY || HAL_I2C_GetState(master) != HAL_I2C_STATE_READY) {
        console_write("  I2C: HAL-Controller nicht bereit. Board resetten und erneut versuchen.\r\n");
        return;
    }

    memset(i2c_rx_buffer, 0, sizeof(i2c_rx_buffer));
    memset(i2c_reply_buffer, 0, sizeof(i2c_reply_buffer));
    i2c_rx_complete = 0U;
    i2c_tx_armed = 0U;
    i2c_tx_complete = 0U;
    i2c_slave_error = 0U;
    active_i2c_slave = slave;
    if (!status_ok(HAL_I2C_Slave_Receive_IT(slave, i2c_rx_buffer, sizeof(i2c_rx_buffer)), "I2C slave receive start")) return;
    if (!status_ok(HAL_I2C_Master_Transmit(master, (uint16_t)(0x42U << 1U), (uint8_t *)tx, sizeof(tx), 1000U), "I2C master senden")) {
        console_printf("  %s: keine Slave-Antwort. SDA/SCL-Jumper und Pinmapping pruefen.\r\n", master_name);
        active_i2c_slave = NULL;
        return;
    }

    uint32_t started = HAL_GetTick();
    while (!i2c_rx_complete && !i2c_slave_error && (HAL_GetTick() - started) < 1000U) { }
    if (!i2c_rx_complete || i2c_slave_error) {
        console_printf("  %s: Slave hat keine vollstaendige Anfrage verarbeitet.\r\n", master_name);
        active_i2c_slave = NULL;
        return;
    }
    if (!status_ok(HAL_I2C_Slave_Transmit_IT(slave, i2c_reply_buffer, sizeof(i2c_reply_buffer)), "I2C slave antwort start")) {
        active_i2c_slave = NULL;
        return;
    }
    i2c_tx_armed = 1U;
    if (!status_ok(HAL_I2C_Master_Receive(master, (uint16_t)(0x42U << 1U), rx, sizeof(rx), 1000U), "I2C master empfangen")) {
        console_printf("  %s: Lesen vom Slave fehlgeschlagen.\r\n", master_name);
        active_i2c_slave = NULL;
        return;
    }
    started = HAL_GetTick();
    while (!i2c_tx_complete && !i2c_slave_error && (HAL_GetTick() - started) < 1000U) { }
    active_i2c_slave = NULL;

    uint8_t expected[sizeof(tx)];
    for (size_t i = 0U; i < sizeof(tx); ++i) expected[i] = (uint8_t)(tx[i] ^ 0xA5U);
    if (!i2c_slave_error && i2c_tx_armed && i2c_tx_complete && memcmp(i2c_rx_buffer, tx, sizeof(tx)) == 0 &&
        memcmp(rx, expected, sizeof(expected)) == 0) {
        console_printf("  %s <-> %s: PASS, Schreibdaten empfangen und Antwortbytes korrekt zurueckgelesen.\r\n",
                       master_name, slave_name);
    } else {
        console_printf("  %s <-> %s: FAIL, RX=%u, TX-Callback=%u, Slavefehler=%u.\r\n",
                       master_name, slave_name, (unsigned)i2c_rx_complete,
                       (unsigned)i2c_tx_complete, (unsigned)i2c_slave_error);
    }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c != active_i2c_slave) return;
    for (size_t i = 0U; i < sizeof(i2c_rx_buffer); ++i) {
        i2c_reply_buffer[i] = (uint8_t)(i2c_rx_buffer[i] ^ 0xA5U);
    }
    i2c_rx_complete = 1U;
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == active_i2c_slave) i2c_tx_complete = 1U;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == active_i2c_slave) i2c_slave_error = 1U;
}

void I2C1_EV_IRQHandler(void) { HAL_I2C_EV_IRQHandler(&hi2c1); }
void I2C1_ER_IRQHandler(void) { HAL_I2C_ER_IRQHandler(&hi2c1); }
void I2C3_EV_IRQHandler(void) { HAL_I2C_EV_IRQHandler(&hi2c3); }
void I2C3_ER_IRQHandler(void) { HAL_I2C_ER_IRQHandler(&hi2c3); }

static uint8_t tim1_pwm_init(void)
{
    TIM_OC_InitTypeDef oc = {0};
    gpio_config_af(GPIOB, GPIO_PIN_13 | GPIO_PIN_15, GPIO_AF1_TIM1);
    gpio_config_af(GPIOA, GPIO_PIN_9 | GPIO_PIN_11, GPIO_AF1_TIM1);
    __HAL_RCC_TIM1_CLK_ENABLE();
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 15U;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 999U;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0U;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (!status_ok(HAL_TIM_PWM_Init(&htim1), "TIM1 PWM init")) return 0U;
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 500U;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    oc.OCIdleState = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (!status_ok(HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_1), "TIM1 CH1 config")) return 0U;
    if (!status_ok(HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_2), "TIM1 CH2 config")) return 0U;
    if (!status_ok(HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_3), "TIM1 CH3 config")) return 0U;
    if (!status_ok(HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_4), "TIM1 CH4 config")) return 0U;
    return 1U;
}

static uint8_t tim3_pwm_init(void)
{
    TIM_OC_InitTypeDef oc = {0};
    gpio_config_af(GPIOC, GPIO_PIN_8, GPIO_AF2_TIM3);
    __HAL_RCC_TIM3_CLK_ENABLE();
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 15U;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 999U;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (!status_ok(HAL_TIM_PWM_Init(&htim3), "TIM3 PWM init")) return 0U;
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 500U;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    if (!status_ok(HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_3), "TIM3 CH3 config")) return 0U;
    return 1U;
}

static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = (HAL_RCC_GetHCLKFreq() / 1000000U) * us;
    while ((uint32_t)(DWT->CYCCNT - start) < ticks) { }
}

static uint8_t sample_pwm(GPIO_TypeDef *port, uint16_t pin)
{
    uint16_t highs = 0U;
    uint16_t lows = 0U;
    for (uint16_t i = 0U; i < 120U; ++i) {
        if (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) ++highs;
        else ++lows;
        delay_us(100U);
    }
    return (highs > 0U && lows > 0U) ? 1U : 0U;
}

static void test_pwm(void)
{
    gpio_config_input(FEEDBACK_PORT, FEEDBACK_PIN, GPIO_PULLDOWN);
    if (!tim1_pwm_init() || !tim3_pwm_init()) return;
    console_write("\r\nPWM-Test: 1 kHz / ca. 50 %, jeder Ausgang einzeln. Feedback jeweils nach CN7-32 (PA4).\r\n");
    console_write("Mit Oszilloskop direkt am PWM-Pin messen oder Jumper zum Feedback-Eingang stecken.\r\n");
    for (size_t i = 0U; i < sizeof(pwm_outputs) / sizeof(pwm_outputs[0]); ++i) {
        const PwmRow *row = &pwm_outputs[i];
        char instruction[180];
        (void)snprintf(instruction, sizeof(instruction), "%s (%s) -> CN7-32 (PA4)", row->name, row->cn);
        if (!wait_go(instruction)) continue;
        HAL_StatusTypeDef start_status;
        if (row->timer3) start_status = HAL_TIM_PWM_Start(&htim3, row->channel);
        else if (row->complementary) start_status = HAL_TIMEx_PWMN_Start(&htim1, row->channel);
        else start_status = HAL_TIM_PWM_Start(&htim1, row->channel);
        if (!status_ok(start_status, "PWM start")) continue;
        HAL_Delay(2U);
        uint8_t passed = sample_pwm(FEEDBACK_PORT, FEEDBACK_PIN);
        if (row->timer3) (void)HAL_TIM_PWM_Stop(&htim3, row->channel);
        else if (row->complementary) (void)HAL_TIMEx_PWMN_Stop(&htim1, row->channel);
        else (void)HAL_TIM_PWM_Stop(&htim1, row->channel);
        console_printf("  %s: %s (Rueckmeldung muss HIGH und LOW gesehen haben)\r\n",
                       row->name, passed ? "PASS" : "FAIL");
    }
    console_write("Hinweis: PC8 wird hier als 1-kHz-Testsignal geprueft, nicht als WS2811-Datenstrom.\r\n");
}

static uint8_t wait_tim2_capture(uint32_t *capture, uint32_t timeout_ms)
{
    uint32_t started = HAL_GetTick();
    while (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC1) == RESET) {
        if ((HAL_GetTick() - started) >= timeout_ms) return 0U;
    }
    *capture = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC1);
    return 1U;
}

static void test_flow_input(void)
{
    TIM_IC_InitTypeDef capture = {0};
    uint32_t first = 0U, second = 0U;
    console_write("\r\nFlowmeter-Eingang: Test-PWM von PA9 CN10-21 nach PA15 CN10-11.\r\n");
    if (!tim1_pwm_init()) return;
    gpio_config_af(GPIOA, GPIO_PIN_15, GPIO_AF1_TIM2);
    __HAL_RCC_TIM2_CLK_ENABLE();
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 15U;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFFFFFFU;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (!status_ok(HAL_TIM_IC_Init(&htim2), "TIM2 input-capture init")) return;
    capture.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    capture.ICSelection = TIM_ICSELECTION_DIRECTTI;
    capture.ICPrescaler = TIM_ICPSC_DIV1;
    capture.ICFilter = 0U;
    if (!status_ok(HAL_TIM_IC_ConfigChannel(&htim2, &capture, TIM_CHANNEL_1), "TIM2 CH1 config")) return;
    if (!wait_go("Jumper PA9 / TIM1_CH2 CN10-21 auf PA15 / TIM2_CH1 CN10-11 stecken.")) return;
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC1);
    if (!status_ok(HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_1), "TIM2 capture start")) return;
    if (!status_ok(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2), "TIM1 test PWM start")) return;
    uint8_t got_first = wait_tim2_capture(&first, 1000U);
    uint8_t got_second = got_first ? wait_tim2_capture(&second, 1000U) : 0U;
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIM_IC_Stop(&htim2, TIM_CHANNEL_1);
    if (!got_first || !got_second) {
        console_write("  Flow input: FAIL, keine Flanken erkannt; Jumper/Pins pruefen.\r\n");
        return;
    }
    uint32_t ticks = second - first;
    if (ticks == 0U) {
        console_write("  Flow input: FAIL, ungueltige Capture-Differenz.\r\n");
        return;
    }
    uint32_t frequency = 1000000U / ticks;
    console_printf("  Flow input: %s, gemessen ca. %lu Hz (Soll ca. 1000 Hz).\r\n",
                   (frequency >= 900U && frequency <= 1100U) ? "PASS" : "CHECK",
                   (unsigned long)frequency);
}

static void watch_inputs(void)
{
    int previous = -1;
    uint8_t ch;
    console_write("\r\nEingangsmonitor: Statuswechsel werden angezeigt. Eingangs-Pins einzeln mit GND/3V3 testen; q beendet.\r\n");
    for (size_t i = 0U; i < sizeof(gpio_inputs) / sizeof(gpio_inputs[0]); ++i) {
        gpio_config_input(gpio_inputs[i].port, gpio_inputs[i].pin, gpio_inputs[i].pull);
    }
    for (;;) {
        int current = 0;
        for (size_t i = 0U; i < sizeof(gpio_inputs) / sizeof(gpio_inputs[0]); ++i) {
            if (HAL_GPIO_ReadPin(gpio_inputs[i].port, gpio_inputs[i].pin) == GPIO_PIN_SET) current |= (1 << i);
        }
        if (current != previous) {
            console_write("  ");
            for (size_t i = 0U; i < sizeof(gpio_inputs) / sizeof(gpio_inputs[0]); ++i) {
                console_printf("%s=%s  ", gpio_inputs[i].cn,
                               (current & (1 << i)) ? "HIGH" : "LOW ");
            }
            console_write("\r\n");
            previous = current;
        }
        if (HAL_UART_Receive(&huart2, &ch, 1U, 40U) == HAL_OK && (ch == 'q' || ch == 'Q')) {
            console_write("Eingangsmonitor beendet.\r\n");
            return;
        }
        HAL_Delay(20U);
    }
}

static void test_led(void)
{
    gpio_config_output(GPIOA, GPIO_PIN_5);
    console_write("\r\nUser LED PA5 blinkt jetzt 8-mal.\r\n");
    for (uint8_t i = 0U; i < 8U; ++i) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        HAL_Delay(250U);
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        HAL_Delay(250U);
    }
    console_write("LED-Test beendet.\r\n");
}

static void run_all(void)
{
    console_write("\r\nStarte die Tests nacheinander. Bei jedem Schritt Jumper nach Anzeige umstecken.\r\n");
    test_gpio_outputs();
    test_spi3();
    test_uart4();
    test_i2c_loop(1U);
    test_i2c_loop(0U);
    test_pwm();
    test_flow_input();
    console_write("\r\nAutomatisierte Schritte fertig. Fuer Taster/Fehlereingaenge jetzt 'inputs' starten.\r\n");
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    dwt_init();
    console_init();
    console_write("Nucleo-F446RE Pinmapping-Test gestartet (USART2 / ST-LINK VCP, 115200 8N1).\r\n");
    console_write("'help' zeigt die Befehle. Vor Tests angeschlossene Treiber/Module entfernen.\r\n");

    char line[64];
    for (;;) {
        console_write("\r\nnucleo> ");
        console_read_line(line, sizeof(line));
        if (line[0] == '\0') continue;
        if (strcmp(line, "help") == 0) print_help();
        else if (strcmp(line, "map") == 0) print_map();
        else if (strcmp(line, "gpio") == 0) test_gpio_outputs();
        else if (strcmp(line, "spi3") == 0) test_spi3();
        else if (strcmp(line, "uart4") == 0) test_uart4();
        else if (strcmp(line, "i2c1") == 0) test_i2c_loop(1U);
        else if (strcmp(line, "i2c3") == 0) test_i2c_loop(0U);
        else if (strcmp(line, "pwm") == 0) test_pwm();
        else if (strcmp(line, "flow") == 0) test_flow_input();
        else if (strcmp(line, "inputs") == 0) watch_inputs();
        else if (strcmp(line, "led") == 0) test_led();
        else if (strcmp(line, "all") == 0) run_all();
        else console_write("Unbekannt. 'help' zeigt die Befehle.\r\n");
    }
}

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();
    clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_0) != HAL_OK) Error_Handler();
}

static void Error_Handler(void)
{
    __disable_irq();
    for (;;) { }
}
