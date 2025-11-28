#pragma once

#include "platform.h"

#ifdef USE_DEBUG_GPIO

#include <stdint.h>

typedef enum {
    DEBUG_GPIO_PIN_PC0 = 0,
    DEBUG_GPIO_PIN_PC1,
    DEBUG_GPIO_PIN_PC2,
    DEBUG_GPIO_PIN_PC3,
    DEBUG_GPIO_PIN_COUNT
} debugGpioPin_e;

void debugGpioInit(void);
void debugGpioSetHigh(debugGpioPin_e pin);
void debugGpioSetLow(debugGpioPin_e pin);
void debugGpioToggle(debugGpioPin_e pin);

static inline void debugGpioPC0High(void) { debugGpioSetHigh(DEBUG_GPIO_PIN_PC0); }
static inline void debugGpioPC0Low(void) { debugGpioSetLow(DEBUG_GPIO_PIN_PC0); }
static inline void debugGpioPC0Toggle(void) { debugGpioToggle(DEBUG_GPIO_PIN_PC0); }

static inline void debugGpioPC1High(void) { debugGpioSetHigh(DEBUG_GPIO_PIN_PC1); }
static inline void debugGpioPC1Low(void) { debugGpioSetLow(DEBUG_GPIO_PIN_PC1); }
static inline void debugGpioPC1Toggle(void) { debugGpioToggle(DEBUG_GPIO_PIN_PC1); }

static inline void debugGpioPC2High(void) { debugGpioSetHigh(DEBUG_GPIO_PIN_PC2); }
static inline void debugGpioPC2Low(void) { debugGpioSetLow(DEBUG_GPIO_PIN_PC2); }
static inline void debugGpioPC2Toggle(void) { debugGpioToggle(DEBUG_GPIO_PIN_PC2); }

static inline void debugGpioPC3High(void) { debugGpioSetHigh(DEBUG_GPIO_PIN_PC3); }
static inline void debugGpioPC3Low(void) { debugGpioSetLow(DEBUG_GPIO_PIN_PC3); }
static inline void debugGpioPC3Toggle(void) { debugGpioToggle(DEBUG_GPIO_PIN_PC3); }

#else

#define debugGpioInit()                do {} while (0)
#define debugGpioSetHigh(pin)          do { (void)(pin); } while (0)
#define debugGpioSetLow(pin)           do { (void)(pin); } while (0)
#define debugGpioToggle(pin)           do { (void)(pin); } while (0)
#define debugGpioPC0High()             do {} while (0)
#define debugGpioPC0Low()              do {} while (0)
#define debugGpioPC0Toggle()           do {} while (0)
#define debugGpioPC1High()             do {} while (0)
#define debugGpioPC1Low()              do {} while (0)
#define debugGpioPC1Toggle()           do {} while (0)
#define debugGpioPC2High()             do {} while (0)
#define debugGpioPC2Low()              do {} while (0)
#define debugGpioPC2Toggle()           do {} while (0)
#define debugGpioPC3High()             do {} while (0)
#define debugGpioPC3Low()              do {} while (0)
#define debugGpioPC3Toggle()           do {} while (0)

#endif

