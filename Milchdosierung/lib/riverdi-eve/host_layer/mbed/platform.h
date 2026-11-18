/**
 * @file platform.h
 * @brief C-compatible Riverdi EVE host interface for Mbed OS.
 */
#ifndef RIVERDI_EVE_MBED_PLATFORM_H
#define RIVERDI_EVE_MBED_PLATFORM_H

#include <stdint.h>
#include <stdarg.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef TRUE
#define TRUE (1)
#endif
#ifndef FALSE
#define FALSE (0)
#endif

typedef char bool_t;
typedef char char8_t;
typedef unsigned char uchar8_t;
typedef signed char schar8_t;
typedef float float_t;

#include "modules.h"
#include "Gpu_Hal.h"
#include "Gpu.h"
#include "CoPro_Cmds.h"
#include "Hal_Utils.h"

/* Logical signals; the actual pins come from MilkMatePinMap.h. */
typedef enum {
    GPIO_CS,
    GPIO_PD,
    GPIO_INT
} gpio_name;

typedef enum {
    GPIO_LOW = 0,
    GPIO_HIGH = 1
} gpio_val;

/* Zero-initialize Gpu_Hal_Context_t before opening it. Use from thread context
 * and serialize complete EVE operations when sharing a context between threads.
 */
bool_t platform_init(Gpu_HalInit_t *halinit);
void platform_sleep_ms(uint32_t ms);

bool_t platform_spi_init(Gpu_Hal_Context_t *host);
void platform_spi_deinit(Gpu_Hal_Context_t *host);

uchar8_t platform_spi_send_recv_byte(Gpu_Hal_Context_t *host, uchar8_t data, uint32_t opt);
uint16_t platform_spi_send_data(Gpu_Hal_Context_t *host, uchar8_t *data, uint16_t size, uint32_t opt);
void platform_spi_recv_data(Gpu_Hal_Context_t *host, uchar8_t *data, uint16_t size, uint32_t opt);

bool_t platform_gpio_init(Gpu_Hal_Context_t *host, gpio_name gpio);
bool_t platform_gpio_value(Gpu_Hal_Context_t *host, gpio_name gpio, gpio_val value);

/** Callback for the falling edge of INT_N. Runs in ISR context: only signal
 * the display thread here; perform SPI transfers and EVE processing there.
 */
typedef void (*platform_interrupt_callback_t)(void *context);

/** Register after Gpu_Hal_Open(), before enabling interrupt sources in EVE.
 * Pass a null callback to detach. Registration/replacement/close must be
 * serialized by the caller in thread context. The callback context must remain
 * valid until detached or Gpu_Hal_Close(). Returns FALSE for an unopened host.
 */
bool_t platform_set_interrupt_callback(Gpu_Hal_Context_t *host,
                                     platform_interrupt_callback_t callback,
                                     void *context);

#ifdef __cplusplus
}
#endif

#endif /* RIVERDI_EVE_MBED_PLATFORM_H */
