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

static inline void debugLine5High(void) { debugGpioSetHigh(DEBUG_GPIO_PIN_PC0); }
static inline void debugLine5Low(void) { debugGpioSetLow(DEBUG_GPIO_PIN_PC0); }
static inline void debugLine5Toggle(void) { debugGpioToggle(DEBUG_GPIO_PIN_PC0); }

static inline void debugLine4High(void) { debugGpioSetHigh(DEBUG_GPIO_PIN_PC1); }
static inline void debugLine4Low(void) { debugGpioSetLow(DEBUG_GPIO_PIN_PC1); }
static inline void debugLine4Toggle(void) { debugGpioToggle(DEBUG_GPIO_PIN_PC1); }

static inline void debugLine6High(void) { debugGpioSetHigh(DEBUG_GPIO_PIN_PC2); }
static inline void debugLine6Low(void) { debugGpioSetLow(DEBUG_GPIO_PIN_PC2); }
static inline void debugLine6Toggle(void) { debugGpioToggle(DEBUG_GPIO_PIN_PC2); }

static inline void debugLine7High(void) { debugGpioSetHigh(DEBUG_GPIO_PIN_PC3); }
static inline void debugLine7Low(void) { debugGpioSetLow(DEBUG_GPIO_PIN_PC3); }
static inline void debugLine7Toggle(void) { debugGpioToggle(DEBUG_GPIO_PIN_PC3); }

#else

#define debugGpioInit()                do {} while (0)
#define debugGpioSetHigh(pin)          do { (void)(pin); } while (0)
#define debugGpioSetLow(pin)           do { (void)(pin); } while (0)
#define debugGpioToggle(pin)           do { (void)(pin); } while (0)
#define debugLine5High()             do {} while (0)
#define debugLine5Low()              do {} while (0)
#define debugLine5Toggle()           do {} while (0)
#define debugLine4High()             do {} while (0)
#define debugLine4Low()              do {} while (0)
#define debugLine4Toggle()           do {} while (0)
#define debugLine6High()             do {} while (0)
#define debugLine6Low()              do {} while (0)
#define debugLine6Toggle()           do {} while (0)
#define debugLine7High()             do {} while (0)
#define debugLine7Low()              do {} while (0)
#define debugLine7Toggle()           do {} while (0)

#endif

