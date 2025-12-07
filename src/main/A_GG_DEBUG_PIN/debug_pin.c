#include "platform.h"
#include <stdint.h>

#include "debug_pin.h"
#ifdef USE_DEBUG_GPIO

#include "drivers/io.h"
#include "drivers/io_impl.h"

typedef struct {
    IO_t io;
} debugGpioPort_t;

static debugGpioPort_t debugGpioPorts[DEBUG_GPIO_PIN_COUNT];

static const ioTag_t debugGpioTags[DEBUG_GPIO_PIN_COUNT] = {
    IO_TAG(PC0),
    IO_TAG(PC1),
    IO_TAG(PC2),
    IO_TAG(PC3),
};

void debugGpioInit(void)
{
    for (unsigned i = 0; i < DEBUG_GPIO_PIN_COUNT; i++) {
        const ioTag_t tag = debugGpioTags[i];
        IO_t pin = IOGetByTag(tag);
        debugGpioPorts[i].io = IO_NONE;
        if (!pin) {
            continue;
        }

        IOInit(pin, OWNER_SYSTEM, RESOURCE_INDEX(i));
        IOConfigGPIO(pin, IOCFG_OUT_PP);
        IOHi(pin);
        debugGpioPorts[i].io = pin;
    }
}

static IO_t debugGpioGetIo(debugGpioPin_e pin)
{
    if (pin >= DEBUG_GPIO_PIN_COUNT) {
        return IO_NONE;
    }
    return debugGpioPorts[pin].io;
}

void debugGpioSetHigh(debugGpioPin_e pin)
{
    const IO_t io = debugGpioGetIo(pin);
    if (io) {
        IOHi(io);
    }
}

void debugGpioSetLow(debugGpioPin_e pin)
{
    const IO_t io = debugGpioGetIo(pin);
    if (io) {
        IOLo(io);
    }
}

void debugGpioToggle(debugGpioPin_e pin)
{
    const IO_t io = debugGpioGetIo(pin);
    if (io) {
        IOToggle(io);
    }
}

#else

// Empty implementations when USE_DEBUG_GPIO is not defined
// These must exist even when the macro is not defined to satisfy linker
// Undefine macros first to allow function definitions
#undef debugGpioInit
#undef debugGpioSetHigh
#undef debugGpioSetLow
#undef debugGpioToggle

void debugGpioInit(void)
{
    // Empty implementation
}

void debugGpioSetHigh(debugGpioPin_e pin)
{
    UNUSED(pin);
}

void debugGpioSetLow(debugGpioPin_e pin)
{
    UNUSED(pin);
}

void debugGpioToggle(debugGpioPin_e pin)
{
    UNUSED(pin);
}

#endif

