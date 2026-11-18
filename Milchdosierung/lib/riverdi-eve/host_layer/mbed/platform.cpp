/**
 * @file platform.cpp
 * @brief Riverdi EVE SPI/GPIO transport using the MilkMate pins and Mbed OS.
 */
#include "mbed.h"
#include "MilkMatePinMap.h"
#include <chrono>
#include <new>

/* Include Mbed/C++ headers before the EVE headers, which define common macros. */
#include "platform.h"

namespace {
constexpr int SPI_FREQUENCY_HZ = 1000000;

struct MbedHost {
    mbed::SPI spi;
    mbed::DigitalOut chip_select;
    mbed::DigitalOut power_down;
    mbed::InterruptIn interrupt;
    platform_interrupt_callback_t interrupt_callback = nullptr;
    void *interrupt_context = nullptr;
    bool selected = false;

    MbedHost()
        : spi(milk_pins::DISPLAY_MOSI, milk_pins::DISPLAY_MISO, milk_pins::DISPLAY_SCK),
          chip_select(milk_pins::DISPLAY_CS, 1),
          power_down(milk_pins::DISPLAY_RESET, 1),
          interrupt(milk_pins::DISPLAY_INT, PullUp)
    {
    }

    void set_interrupt_callback(platform_interrupt_callback_t callback, void *context)
    {
        // Keep the callback and its context consistent with respect to the ISR.
        mbed::CriticalSectionLock lock;
        interrupt.fall(nullptr);
        interrupt_callback = callback;
        interrupt_context = callback ? context : nullptr;
        if (callback) {
            interrupt.fall(mbed::callback(this, &MbedHost::on_interrupt));
        }
    }

    void on_interrupt()
    {
        if (interrupt_callback) {
            interrupt_callback(interrupt_context);
        }
    }

    void deselect()
    {
        chip_select = 1;
        if (selected) {
            selected = false;
            spi.deselect();
        }
    }
};

MbedHost *get_host(Gpu_Hal_Context_t *host)
{
    return host ? static_cast<MbedHost *>(host->hal_handle) : nullptr;
}

MbedHost *create_host(Gpu_Hal_Context_t *host)
{
    if (!host) {
        return nullptr;
    }
    if (!host->hal_handle) {
        host->hal_handle = new (std::nothrow) MbedHost;
    }
    return get_host(host);
}

void transfer_error(Gpu_Hal_Context_t *host)
{
    if (host) {
        host->status = GPU_HAL_STATUS_ERROR;
    }
}
} // namespace

bool_t platform_init(Gpu_HalInit_t *halinit)
{
    if (!halinit) {
        return FALSE;
    }
    // Mbed already initialized the MCU, clocks and RTOS.
    halinit->TotalChannelNum = 1;
    return TRUE;
}

void platform_sleep_ms(uint32_t ms)
{
    rtos::ThisThread::sleep_for(std::chrono::milliseconds(ms));
}

bool_t platform_spi_init(Gpu_Hal_Context_t *host)
{
    MbedHost *hardware = create_host(host);
    if (!hardware) {
        return FALSE;
    }
    hardware->spi.format(8, 0);
    hardware->spi.frequency(SPI_FREQUENCY_HZ);
    hardware->spi.set_default_write_value(0);
    host->hal_config.spi_clockrate_khz = SPI_FREQUENCY_HZ / 1000;
    return TRUE;
}

void platform_spi_deinit(Gpu_Hal_Context_t *host)
{
    MbedHost *hardware = get_host(host);
    if (hardware) {
        hardware->set_interrupt_callback(nullptr, nullptr);
        hardware->deselect();
        hardware->power_down = 0;
        delete hardware;
        host->hal_handle = nullptr;
    }
}

uchar8_t platform_spi_send_recv_byte(Gpu_Hal_Context_t *host, uchar8_t data, uint32_t /* opt */)
{
    MbedHost *hardware = get_host(host);
    if (!hardware) {
        transfer_error(host);
        return 0;
    }
    const int result = hardware->spi.write(data);
    if (result < 0) {
        transfer_error(host);
        return 0;
    }
    return static_cast<uchar8_t>(result);
}

uint16_t platform_spi_send_data(Gpu_Hal_Context_t *host, uchar8_t *data, uint16_t size, uint32_t /* opt */)
{
    if (size == 0) {
        return 0;
    }
    MbedHost *hardware = get_host(host);
    if (!hardware || !data) {
        transfer_error(host);
        return 0;
    }
    const int transferred = hardware->spi.write(reinterpret_cast<const char *>(data), size, nullptr, 0);
    if (transferred != size) {
        transfer_error(host);
        return 0;
    }
    return size;
}

void platform_spi_recv_data(Gpu_Hal_Context_t *host, uchar8_t *data, uint16_t size, uint32_t /* opt */)
{
    if (size == 0) {
        return;
    }
    MbedHost *hardware = get_host(host);
    if (!hardware || !data) {
        transfer_error(host);
        return;
    }
    if (hardware->spi.write(nullptr, 0, reinterpret_cast<char *>(data), size) != size) {
        transfer_error(host);
    }
}

bool_t platform_gpio_init(Gpu_Hal_Context_t *host, gpio_name gpio)
{
    if (!host) {
        return FALSE;
    }
    if (gpio != GPIO_CS && gpio != GPIO_PD && gpio != GPIO_INT) {
        return FALSE;
    }
    return create_host(host) ? TRUE : FALSE;
}

bool_t platform_gpio_value(Gpu_Hal_Context_t *host, gpio_name gpio, gpio_val value)
{
    MbedHost *hardware = get_host(host);
    if (!hardware || (value != GPIO_LOW && value != GPIO_HIGH)) {
        return FALSE;
    }
    switch (gpio) {
        case GPIO_CS:
            if (value == GPIO_LOW) {
                if (!hardware->selected) {
                    // Hold the SPI bus across address, dummy and payload bytes.
                    hardware->spi.select();
                    hardware->selected = true;
                }
                hardware->chip_select = 0;
            } else {
                hardware->deselect();
            }
            return TRUE;
        case GPIO_PD:
            hardware->power_down = value;
            return TRUE;
        // INT_N is an input and cannot be driven through this API.
        default:
            return FALSE;
    }
}

bool_t platform_set_interrupt_callback(Gpu_Hal_Context_t *host,
                                      platform_interrupt_callback_t callback,
                                      void *context)
{
    MbedHost *hardware = get_host(host);
    if (!hardware) {
        return FALSE;
    }
    hardware->set_interrupt_callback(callback, context);
    return TRUE;
}
