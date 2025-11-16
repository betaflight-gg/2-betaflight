#include "platform.h"

#ifdef USE_CONFIG_LOGGER

#include <stdbool.h>
#include <stddef.h>
#include <stdarg.h>
#include <string.h>

#include "common/printf.h"
#include "common/time.h"
#include "common/sensor_alignment.h"

#include "drivers/serial.h"
#include "drivers/time.h"

#include "io/serial.h"

#include "flight/imu.h"
#include "flight/pid.h"

#include "sensors/acceleration.h"
#include "sensors/boardalignment.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

#include "pg/dyn_notch.h"
#include "pg/gyrodev.h"
#include "pg/rpm_filter.h"

#include "config_log/config_log.h"

#define CONFIG_LOG_PAYLOAD_LEN 192
#define CONFIG_LOG_LINE_LEN    256

typedef struct {
    char *buffer;
    size_t length;
    size_t capacity;
} configLogBuffer_t;

static serialPort_t *configLogPort;
static uint32_t configLogCounter;
static bool configLogReady;
static uint16_t configLogImuRegIndex;

static void configLogBufferPutc(void *p, char c)
{
    configLogBuffer_t *ctx = (configLogBuffer_t *)p;

    if (!ctx || !ctx->buffer) {
        return;
    }

    if (ctx->length + 1 >= ctx->capacity) {
        return;
    }

    ctx->buffer[ctx->length++] = c;
}

static void configLogBlankLine(void)
{
    if (!configLogReady || !configLogPort) {
        return;
    }

    const char nl[] = "\r\n";
    serialWriteBuf(configLogPort, (const uint8_t *)nl, sizeof(nl) - 1);
}

static void configLogOutput(const char *section, const char *payload)
{
    if (!configLogReady || !configLogPort || !section || !payload) {
        return;
    }

    char line[CONFIG_LOG_LINE_LEN];
    const timeUs_t now = micros();
    configLogCounter++;

    const int len = tfp_sprintf(
        line,
        "[CFG][%s][count=%lu][time=%luus]%s\r\n",
        section,
        (unsigned long)configLogCounter,
        (unsigned long)now,
        payload
    );

    if (len > 0) {
        serialWriteBuf(configLogPort, (const uint8_t *)line, len);
        while (!isSerialTransmitBufferEmpty(configLogPort)) {
            /* wait for TX completing to keep debug order */
        }
    }
}

void configLogMessage(const char *section, const char *fmt, ...)
{
    if (!configLogReady || !fmt) {
        return;
    }

    char payload[CONFIG_LOG_PAYLOAD_LEN];
    configLogBuffer_t bufferCtx = {
        .buffer = payload,
        .length = 0,
        .capacity = sizeof(payload)
    };

    payload[0] = '\0';

    va_list va;
    va_start(va, fmt);
    tfp_format(&bufferCtx, configLogBufferPutc, fmt, va);
    va_end(va);

    if (bufferCtx.length >= bufferCtx.capacity) {
        bufferCtx.length = bufferCtx.capacity - 1;
    }
    payload[bufferCtx.length] = '\0';

    configLogOutput(section ? section : "CFG", payload);
}

void configLogInit(void)
{
    if (configLogReady) {
        return;
    }

    configLogPort = openSerialPort(
        SERIAL_PORT_USART2,
        FUNCTION_NONE,
        NULL,
        NULL,
        baudRates[BAUD_57600],
        MODE_TX,
        SERIAL_NOT_INVERTED);

    configLogCounter = 0;
    configLogReady = (configLogPort != NULL);

    if (configLogReady) {
        configLogMessage("LOGGER", "UART1 ready baud=%lu", (unsigned long)baudRates[BAUD_500000]);
    }
}

void configLogRegisterWrite(const char *device, uint16_t reg, uint32_t value)
{
    if (!device) {
        device = "IMU";
    }
    if (configLogImuRegIndex == 0) {
        // 分类头前空一行
        configLogBlankLine();
        configLogMessage("IMU_REG", "==== IMU_REG (sensor registers) ====");
    }
    configLogImuRegIndex++;
    configLogMessage("IMU_REG", "[%u] %s reg=0x%02X value=0x%02X",
        (unsigned)configLogImuRegIndex,
        device,
        reg & 0xFF,
        value & 0xFF);
}

static void configLogDumpGyroDevice(const gyroDeviceConfig_t *cfg, int index)
{
    if (!cfg) {
        return;
    }

    configLogMessage(
        "IMU_ALIGN",
        "gyro%d align=%u cs=0x%04X exti=0x%04X busType=%u spi=%u i2c=%u",
        index,
        cfg->alignment,
        cfg->csnTag,
        cfg->extiTag,
        cfg->busType,
        cfg->spiBus,
        cfg->i2cBus);

    configLogMessage(
        "IMU_ALIGN",
        "gyro%d customAlign roll=%d pitch=%d yaw=%d",
        index,
        cfg->customAlignment.roll,
        cfg->customAlignment.pitch,
        cfg->customAlignment.yaw);
}

void configLogDumpImuAlignment(void)
{
    const boardAlignment_t *board = boardAlignment();
    if (board) {
        configLogMessage(
            "IMU_ALIGN",
            "board roll=%ld pitch=%ld yaw=%ld",
            (long)board->rollDegrees,
            (long)board->pitchDegrees,
            (long)board->yawDegrees);
    }

    for (int i = 0; i < GYRO_COUNT; i++) {
        configLogDumpGyroDevice(gyroDeviceConfig(i), i);
    }
}

void configLogDumpImuCalibration(void)
{
    uint16_t idx = 0;

    configLogBlankLine();
    configLogMessage("IMU", "==== IMU CONFIG ====");

#ifdef USE_ACC
    const accelerometerConfig_t *accCfg = accelerometerConfig();
    if (accCfg) {
        configLogMessage(
            "IMU",
            "[%u] acc lpf=%uHz highFSR=%u zero=[%d,%d,%d,%d] trims=[%d,%d]",
            (unsigned)++idx,
            accCfg->acc_lpf_hz,
            accCfg->acc_high_fsr,
            accCfg->accZero.values.roll,
            accCfg->accZero.values.pitch,
            accCfg->accZero.values.yaw,
            accCfg->accZero.values.calibrationCompleted,
            accCfg->accelerometerTrims.values.roll,
            accCfg->accelerometerTrims.values.pitch);
    }
#endif

    const gyroConfig_t *gyroCfg = gyroConfig();
    if (gyroCfg) {
        configLogMessage(
            "IMU",
            "[%u] gyro hw_lpf=%u highFSR=%u calDur=%u moveThr=%u offsetYaw=%d enabledMask=0x%02X",
            (unsigned)++idx,
            gyroCfg->gyro_hardware_lpf,
            gyroCfg->gyro_high_fsr,
            gyroCfg->gyroCalibrationDuration,
            gyroCfg->gyroMovementCalibrationThreshold,
            gyroCfg->gyro_offset_yaw,
            gyroCfg->gyro_enabled_bitmask);

        configLogMessage(
            "IMU",
            "[%u] gyro lpf1 type=%u static=%uHz dyn=[%u,%u] expo=%u lpf2 type=%u static=%uHz",
            (unsigned)++idx,
            gyroCfg->gyro_lpf1_type,
            gyroCfg->gyro_lpf1_static_hz,
            gyroCfg->gyro_lpf1_dyn_min_hz,
            gyroCfg->gyro_lpf1_dyn_max_hz,
            gyroCfg->gyro_lpf1_dyn_expo,
            gyroCfg->gyro_lpf2_type,
            gyroCfg->gyro_lpf2_static_hz);

        configLogMessage(
            "IMU",
            "[%u] gyro notch1=%u/%u Hz notch2=%u/%u Hz simplified=%u multi=%u",
            (unsigned)++idx,
            gyroCfg->gyro_soft_notch_hz_1,
            gyroCfg->gyro_soft_notch_cutoff_1,
            gyroCfg->gyro_soft_notch_hz_2,
            gyroCfg->gyro_soft_notch_cutoff_2,
            gyroCfg->simplified_gyro_filter,
            gyroCfg->simplified_gyro_filter_multiplier);

        // 当前 gyro 校准零偏（注意：在系统刚启动且尚未完成校准时，这些值通常是 0）
        if (GYRO_COUNT > 0) {
            const gyroSensor_t *g0 = &gyro.gyroSensor[0];
            configLogMessage(
                "IMU",
                "[%u] gyro0 zero=[%d,%d,%d]",
                (unsigned)++idx,
                (int)g0->gyroDev.gyroZero[X],
                (int)g0->gyroDev.gyroZero[Y],
                (int)g0->gyroDev.gyroZero[Z]);
        }
    }

    const imuConfig_t *imuCfg = imuConfig();
    if (imuCfg) {
        configLogMessage(
            "IMU",
            "[%u] imu kp=%u ki=%u smallAngle=%u processDenom=%u magDecl=%d",
            (unsigned)++idx,
            imuCfg->imu_dcm_kp,
            imuCfg->imu_dcm_ki,
            imuCfg->small_angle,
            imuCfg->imu_process_denom,
            imuCfg->mag_declination);
    }
}

void configLogDumpFilterConfig(void)
{
    uint16_t idx = 0;

    configLogBlankLine();
    configLogMessage("FILTER", "==== FILTER CONFIG ====");

    const dynNotchConfig_t *dynCfg = dynNotchConfig();
    if (dynCfg) {
        const bool dynEnabled = dynCfg->dyn_notch_count > 0;
        configLogMessage(
            "FILTER",
            "[%u] dyn_notch enabled=%d count=%u min=%uHz max=%uHz q=%u",
            (unsigned)++idx,
            dynEnabled,
            dynCfg->dyn_notch_count,
            dynCfg->dyn_notch_min_hz,
            dynCfg->dyn_notch_max_hz,
            dynCfg->dyn_notch_q);
    }

    const rpmFilterConfig_t *rpmCfg = rpmFilterConfig();
    if (rpmCfg) {
        const bool rpmEnabled = rpmCfg->rpm_filter_harmonics > 0;
        configLogMessage(
            "FILTER",
            "[%u] rpm_filter enabled=%d harmonics=%u min=%uHz fade=%uHz q=%u lpf=%uHz weights=[%u,%u,%u]",
            (unsigned)++idx,
            rpmEnabled,
            rpmCfg->rpm_filter_harmonics,
            rpmCfg->rpm_filter_min_hz,
            rpmCfg->rpm_filter_fade_range_hz,
            rpmCfg->rpm_filter_q,
            rpmCfg->rpm_filter_lpf_hz,
            rpmCfg->rpm_filter_weights[0],
            rpmCfg->rpm_filter_weights[1],
            rpmCfg->rpm_filter_weights[2]);
    }

    const gyroConfig_t *gyroCfg = gyroConfig();
    if (gyroCfg) {
        const bool lpf1Enabled = gyroCfg->gyro_lpf1_static_hz > 0 || gyroCfg->gyro_lpf1_dyn_max_hz > 0;
        const bool lpf2Enabled = gyroCfg->gyro_lpf2_static_hz > 0;
        const bool softNotch1Enabled = gyroCfg->gyro_soft_notch_hz_1 > 0 && gyroCfg->gyro_soft_notch_cutoff_1 > 0;
        const bool softNotch2Enabled = gyroCfg->gyro_soft_notch_hz_2 > 0 && gyroCfg->gyro_soft_notch_cutoff_2 > 0;

        configLogMessage(
            "FILTER",
            "[%u] gyro_lpf1 enabled=%d type=%u static=%uHz dyn=[%u,%u] expo=%u",
            (unsigned)++idx,
            lpf1Enabled,
            gyroCfg->gyro_lpf1_type,
            gyroCfg->gyro_lpf1_static_hz,
            gyroCfg->gyro_lpf1_dyn_min_hz,
            gyroCfg->gyro_lpf1_dyn_max_hz,
            gyroCfg->gyro_lpf1_dyn_expo);

        configLogMessage(
            "FILTER",
            "[%u] gyro_lpf2 enabled=%d type=%u static=%uHz",
            (unsigned)++idx,
            lpf2Enabled,
            gyroCfg->gyro_lpf2_type,
            gyroCfg->gyro_lpf2_static_hz);

        configLogMessage(
            "FILTER",
            "[%u] gyro_soft_notch1 enabled=%d freq=%uHz cutoff=%uHz",
            (unsigned)++idx,
            softNotch1Enabled,
            gyroCfg->gyro_soft_notch_hz_1,
            gyroCfg->gyro_soft_notch_cutoff_1);

        configLogMessage(
            "FILTER",
            "[%u] gyro_soft_notch2 enabled=%d freq=%uHz cutoff=%uHz",
            (unsigned)++idx,
            softNotch2Enabled,
            gyroCfg->gyro_soft_notch_hz_2,
            gyroCfg->gyro_soft_notch_cutoff_2);
    }
}

static void configLogDumpPidAxis(const pidProfile_t *profile, pidIndex_e axis, int profileIndex)
{
    if (!profile) {
        return;
    }

    const pidf_t *pid = &profile->pid[axis];
    configLogMessage(
        "PID",
        "profile%d axis=%d PIDFS=[%u,%u,%u,%u,%u] dmax=%u",
        profileIndex,
        axis,
        pid->P,
        pid->I,
        pid->D,
        pid->F,
        pid->S,
        profile->d_max[axis]);
}

void configLogDumpPidProfiles(void)
{
    uint16_t idx = 0;

    configLogBlankLine();
    configLogMessage("PID", "==== PID CONFIG ====");

    const pidConfig_t *pidCfg = pidConfig();
    if (pidCfg) {
        configLogMessage(
            "PID",
            "[%u] pid_process_denom=%u runaway=%u delay=%u throttle=%u",
            (unsigned)++idx,
            pidCfg->pid_process_denom,
            pidCfg->runaway_takeoff_prevention,
            pidCfg->runaway_takeoff_deactivate_delay,
            pidCfg->runaway_takeoff_deactivate_throttle);
    }

    for (int profileIndex = 0; profileIndex < PID_PROFILE_COUNT; profileIndex++) {
        const pidProfile_t *profile = pidProfiles(profileIndex);
        if (!profile) {
            continue;
        }

        // 每个 profile 之间空一行，便于阅读
        configLogBlankLine();

        configLogMessage(
            "PID",
            "[%u] profile%d name=%s yawLPF=%u dtermLPF1 type=%u/%uHz dyn[%u,%u] expo=%u lpf2 type=%u/%uHz",
            (unsigned)++idx,
            profileIndex,
            profile->profileName,
            profile->yaw_lowpass_hz,
            profile->dterm_lpf1_type,
            profile->dterm_lpf1_static_hz,
            profile->dterm_lpf1_dyn_min_hz,
            profile->dterm_lpf1_dyn_max_hz,
            profile->dterm_lpf1_dyn_expo,
            profile->dterm_lpf2_type,
            profile->dterm_lpf2_static_hz);

        configLogMessage(
            "PID",
            "[%u] profile%d dtermNotch=%u/%uHz pidSum=%u yawSum=%u motorLimit=%u itermLimit=%u windup=%u yawLPF=%u",
            (unsigned)++idx,
            profileIndex,
            profile->dterm_notch_hz,
            profile->dterm_notch_cutoff,
            profile->pidSumLimit,
            profile->pidSumLimitYaw,
            profile->motor_output_limit,
            profile->itermLimit,
            profile->itermWindup,
            profile->yaw_lowpass_hz);

        configLogMessage(
            "PID",
            "[%u] profile%d crash dThr=%u gThr=%u setpt=%u time=%u delay=%u limitYaw=%u recovery=%u mode=%u",
            (unsigned)++idx,
            profileIndex,
            profile->crash_dthreshold,
            profile->crash_gthreshold,
            profile->crash_setpoint_threshold,
            profile->crash_time,
            profile->crash_delay,
            profile->crash_limit_yaw,
            profile->crash_recovery_angle,
            profile->crash_recovery);

        configLogMessage(
            "PID",
            "[%u] profile%d antiGravity gain=%u cutoff=%u throttleBoost=%u/%u tpaMode=%u rate=%u bp=%u",
            (unsigned)++idx,
            profileIndex,
            profile->anti_gravity_gain,
            profile->anti_gravity_cutoff_hz,
            profile->throttle_boost,
            profile->throttle_boost_cutoff,
            profile->tpa_mode,
            profile->tpa_rate,
            profile->tpa_breakpoint);

        for (pidIndex_e axis = PID_ROLL; axis <= PID_YAW; axis++) {
            configLogDumpPidAxis(profile, axis, profileIndex);
        }
    }
}

void configLogDumpFullConfig(const char *label)
{
    if (!configLogReady) {
        return;
    }

    if (label && label[0]) {
        configLogMessage("LOGGER", "config dump (%s)", label);
    } else {
        configLogMessage("LOGGER", "config dump");
    }

    configLogDumpImuCalibration();
    configLogDumpImuAlignment();
    configLogDumpFilterConfig();
    configLogDumpPidProfiles();
}

#endif

