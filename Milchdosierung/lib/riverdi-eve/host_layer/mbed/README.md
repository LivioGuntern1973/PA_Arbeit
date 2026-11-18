# Mbed OS host layer

This host layer connects the Riverdi C library to the existing MilkMate pin map.
Mbed manages MCU initialization, clocks and the RTOS. SPI uses 8-bit transfers,
mode 0, MSB first and a conservative 1 MHz clock.

| EVE signal | MilkMate pin | STM32 pin |
| --- | --- | --- |
| MOSI | `DISPLAY_MOSI` | PC_12 (SPI3) |
| MISO | `DISPLAY_MISO` | PC_11 (SPI3) |
| SCK | `DISPLAY_SCK` | PC_10 (SPI3) |
| CS_N | `DISPLAY_CS` | PC_7 |
| PD_N / reset | `DISPLAY_RESET` | PB_2 |
| INT_N | `DISPLAY_INT` | PA_10 (EXTI10, CN10-33) |

The EVE PD_N signal is assumed to be wired to `DISPLAY_RESET`.
`platform_gpio_init(GPIO_INT)` configures the interrupt signal as a `DigitalIn`
with a pull-up. Writing this input fails. The existing EVE layer has no
interrupt-read/callback API; interrupt handling can be added by the display class.
`DISPLAY_DC` is not used by the EVE SPI protocol.

Zero-initialize each context before its first use, for example
`Gpu_Hal_Context_t host = {};`. The transport is allocated on the first GPIO/SPI
initialization and stored in `host.hal_handle`. `Gpu_Hal_Close()` releases it and
leaves CS high and PD_N low. A closed context can be opened again.
Use one open context for the connected display; do not copy an open context.

Call from thread context. A transfer keeps the SPI bus selected and locked until
CS is raised, including address, dummy and payload bytes. The Riverdi command
buffers and FIFO pointers still require the caller to serialize complete EVE
operations when several threads share the display.

The `opt` argument is ignored: the EVE layer controls CS explicitly through
`platform_gpio_value()`. Reads transmit zero-valued dummy bytes. Transfers with
an invalid context or buffer set `host.status` to `GPU_HAL_STATUS_ERROR`.

PlatformIO compiles this host layer together with `eve_layer` and `app_layer`.
The vendor demo, other host targets, HAL copies and startup files are excluded.
Display initialization and drawing remain the application's responsibility;
the existing `main.cpp` only includes the API.
