#pragma once

#include <stdint.h>

#ifdef USE_CONFIG_LOGGER

void configLogInit(void);
void configLogDumpImuCalibration(void);
void configLogDumpImuAlignment(void);
void configLogDumpFilterConfig(void);
void configLogDumpPidProfiles(void);
void configLogDumpFullConfig(const char *label);
void configLogRegisterWrite(const char *device, uint16_t reg, uint32_t value);
void configLogMessage(const char *section, const char *fmt, ...);

#else

#define configLogInit()                do {} while (0)
#define configLogDumpImuCalibration()  do {} while (0)
#define configLogDumpImuAlignment()    do {} while (0)
#define configLogDumpFilterConfig()    do {} while (0)
#define configLogDumpPidProfiles()     do {} while (0)
#define configLogDumpFullConfig(label) do {} while (0)
#define configLogRegisterWrite(a,b,c)  do {} while (0)
#define configLogMessage(a, ...)       do {} while (0)

#endif

