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
`platform_gpio_init(GPIO_INT)` configures PA10 as an `InterruptIn` with a pull-up.
Writing this input fails. The falling-edge handler is attached when the
application registers a callback; no callback is attached initially.
`DISPLAY_DC` is not used by the EVE SPI protocol.

Zero-initialize each context before its first use, for example
`Gpu_Hal_Context_t host = {};`. The transport is allocated on the first GPIO/SPI
initialization and stored in `host.hal_handle`. `Gpu_Hal_Close()` releases it and
detaches the interrupt callback before leaving CS high and PD_N low. A closed
context can be opened again, with no callback registered.
Use one open context for the connected display; do not copy an open context.

Call from thread context. A transfer keeps the SPI bus selected and locked until
CS is raised, including address, dummy and payload bytes. The Riverdi command
buffers and FIFO pointers still require the caller to serialize complete EVE
operations when several threads share the display.

## Interrupt callback

After opening the host, register the display thread's ISR notification callback:

```cpp
// on_eve_interrupt must have the signature: void on_eve_interrupt(void *context).
platform_set_interrupt_callback(&host, on_eve_interrupt, &display_thread);
```

The callback runs on each falling edge of PA10 in ISR context. Its only task
should be to signal the display thread, for example with `Thread::flags_set()`.
Perform SPI transfers and read/clear EVE interrupt status in that thread. The
display class also configures the desired EVE interrupt sources and masks.
Register the callback before enabling these sources so an assertion of INT_N
is not missed while no handler is attached.

The callback receives the supplied context pointer unchanged. Keep that object
alive until the callback is detached or the host is closed. Replace or detach
callbacks from thread context and serialize these operations with host closure.
Pass `nullptr` as the callback to detach it:

```cpp
platform_set_interrupt_callback(&host, nullptr, nullptr);
```

Registration on a null or unopened host returns `FALSE`. Callback replacement
uses a critical section so an ISR cannot observe a mismatched function/context
pair. No SPI or application work is performed by the host ISR itself.

The `opt` argument is ignored: the EVE layer controls CS explicitly through
`platform_gpio_value()`. Reads transmit zero-valued dummy bytes. Transfers with
an invalid context or buffer set `host.status` to `GPU_HAL_STATUS_ERROR`.

Check `host.status` after an EVE transfer before using read data. A detected
transfer error is preserved when CS is released and prevents further address
and byte transfers. Close and reopen the host to clear the error.

PlatformIO compiles this host layer together with `eve_layer` and `app_layer`.
The vendor demo, other host targets, HAL copies and startup files are excluded.
Display initialization and drawing remain the application's responsibility;
the existing `main.cpp` only includes the API.
