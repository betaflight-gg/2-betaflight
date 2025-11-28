# IMU方向和姿态旋转处理详解

## 一、IMU坐标系和方向定义

### 1.1 Betaflight标准坐标系

Betaflight使用**NED坐标系**（North-East-Down）作为地球参考坐标系：
- **X轴（Roll轴）**：指向机头方向（前）
- **Y轴（Pitch轴）**：指向右侧（右）
- **Z轴（Yaw轴）**：指向下方（下）

### 1.2 IMU传感器原始坐标系

IMU芯片（如MPU6000、BMI270等）的原始坐标系定义：
- **X轴**：芯片标记的X轴方向
- **Y轴**：芯片标记的Y轴方向  
- **Z轴**：芯片标记的Z轴方向（通常垂直于芯片平面）

**重要**：不同芯片的原始坐标系可能不同，需要通过传感器对齐参数进行转换。

---

## 二、加速度计（Accelerometer）行为

### 2.1 静止状态下的加速度值

当飞行器**水平静止**时：
- **X轴加速度**：≈ 0（水平方向无重力分量）
- **Y轴加速度**：≈ 0（水平方向无重力分量）
- **Z轴加速度**：≈ **+1G**（向下为正，重力向下）

**代码位置：**
```44:54:src/main/sensors/acceleration.c
static inline void alignAccelerometer(void)
{
    switch (acc.dev.accAlign) {
        case ALIGN_CUSTOM:
            alignSensorViaMatrix(&acc.accADC, &acc.dev.rotationMatrix);
            break;
        default:
            alignSensorViaRotation(&acc.accADC, acc.dev.accAlign);
            break;
    }
}
```

加速度计校准时会计算`acc_1G`值（通常为256），用于归一化：
```428:432:src/main/sensors/acceleration_init.c
        // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
        accelerationRuntime.accelerationTrims->raw[X] = (a[X] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES;
        accelerationRuntime.accelerationTrims->raw[Y] = (a[Y] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES;
        accelerationRuntime.accelerationTrims->raw[Z] = (a[Z] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES - acc.dev.acc_1G;
```

### 2.2 各轴加速时的加速度值变化

#### 向X轴正方向加速（机头方向）
- **X轴加速度**：**正值增加**（与加速度方向相同）
- **Y轴加速度**：不变
- **Z轴加速度**：不变（仍受重力影响）

#### 向Y轴正方向加速（右侧）
- **X轴加速度**：不变
- **Y轴加速度**：**正值增加**
- **Z轴加速度**：不变

#### 向Z轴正方向加速（向下）
- **X轴加速度**：不变
- **Y轴加速度**：不变
- **Z轴加速度**：**正值增加**（叠加在1G基础上）

#### 倾斜时的重力分量

当飞行器倾斜时，重力会在各轴产生分量：
- **Roll倾斜**：重力在Y轴产生分量
- **Pitch倾斜**：重力在X轴产生分量
- **Z轴**：始终包含重力分量（1G）

**代码位置：**
```234:248:src/main/flight/imu.c
    // Use measured acceleration vector
    float recipAccNorm = sq(ax) + sq(ay) + sq(az);
    if (useAcc && recipAccNorm > 0.01f) {
        // Normalise accelerometer measurement; useAcc is true when all smoothed acc axes are within 20% of 1G
        recipAccNorm = invSqrt(recipAccNorm);

        ax *= recipAccNorm;
        ay *= recipAccNorm;
        az *= recipAccNorm;

        // Error is sum of cross product between estimated direction and measured direction of gravity
        ex += (ay * rMat.m[2][2] - az * rMat.m[2][1]);
        ey += (az * rMat.m[2][0] - ax * rMat.m[2][2]);
        ez += (ax * rMat.m[2][1] - ay * rMat.m[2][0]);
    }
```

---

## 三、陀螺仪（Gyroscope）行为

### 3.1 陀螺仪正方向定义

陀螺仪测量**角速度**（度/秒或弧度/秒），遵循**右手定则**：
- **X轴（Roll）**：绕X轴旋转，机头向上为正（右滚为正）
- **Y轴（Pitch）**：绕Y轴旋转，机头向上为正（抬头为正）
- **Z轴（Yaw）**：绕Z轴旋转，顺时针为正（从上方看）

### 3.2 静止状态下的陀螺仪值

当飞行器**完全静止**时：
- **X轴角速度**：≈ 0
- **Y轴角速度**：≈ 0
- **Z轴角速度**：≈ 0

**注意**：实际值可能不为0，存在**零偏（gyroZero）**，校准时会自动去除。

**代码位置：**
```204:240:src/main/sensors/gyro.c
STATIC_UNIT_TESTED NOINLINE void performGyroCalibration(gyroSensor_t *gyroSensor, uint8_t gyroMovementCalibrationThreshold)
{
    bool calFailed = false;

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        // Reset g[axis] at start of calibration
        if (isOnFirstGyroCalibrationCycle(&gyroSensor->calibration)) {
            gyroSensor->calibration.sum[axis] = 0.0f;
            devClear(&gyroSensor->calibration.var[axis]);
            // gyroZero is set to zero until calibration complete
            gyroSensor->gyroDev.gyroZero[axis] = 0.0f;
        }

        // Sum up CALIBRATING_GYRO_TIME_US readings
        gyroSensor->calibration.sum[axis] += gyroSensor->gyroDev.gyroADCRaw[axis];
        devPush(&gyroSensor->calibration.var[axis], gyroSensor->gyroDev.gyroADCRaw[axis]);

        if (isOnFinalGyroCalibrationCycle(&gyroSensor->calibration)) {
            const float stddev = devStandardDeviation(&gyroSensor->calibration.var[axis]);
            // DEBUG_GYRO_CALIBRATION_VALUE records the standard deviation of roll
            // into the spare field - debug[3], in DEBUG_GYRO_RAW
            if (axis == X) {
                DEBUG_SET(DEBUG_GYRO_RAW, DEBUG_GYRO_CALIBRATION_VALUE, lrintf(stddev));
            }

            DEBUG_SET(DEBUG_GYRO_CALIBRATION, axis, stddev);

            // check deviation and startover in case the model was moved
            if (gyroMovementCalibrationThreshold && stddev > gyroMovementCalibrationThreshold) {
                calFailed = true;
            } else {
                // please take care with exotic boardalignment !!
                gyroSensor->gyroDev.gyroZero[axis] = gyroSensor->calibration.sum[axis] / gyroCalculateCalibratingCycles();
                if (axis == Z) {
                  gyroSensor->gyroDev.gyroZero[axis] -= ((float)gyroConfig()->gyro_offset_yaw / 100);
                }
            }
        }
    }
```

### 3.3 旋转时的陀螺仪值变化

#### 绕X轴旋转（Roll）
- **X轴角速度**：**正值**（右滚）或**负值**（左滚）
- **Y轴角速度**：≈ 0
- **Z轴角速度**：≈ 0

#### 绕Y轴旋转（Pitch）
- **X轴角速度**：≈ 0
- **Y轴角速度**：**正值**（抬头）或**负值**（低头）
- **Z轴角速度**：≈ 0

#### 绕Z轴旋转（Yaw）
- **X轴角速度**：≈ 0
- **Y轴角速度**：≈ 0
- **Z轴角速度**：**正值**（顺时针）或**负值**（逆时针）

**代码位置：**
```413:422:src/main/sensors/gyro.c
        gyroSensor->gyroDev.gyroADC.x = gyroSensor->gyroDev.gyroADCRaw[X] - gyroSensor->gyroDev.gyroZero[X];
        gyroSensor->gyroDev.gyroADC.y = gyroSensor->gyroDev.gyroADCRaw[Y] - gyroSensor->gyroDev.gyroZero[Y];
        gyroSensor->gyroDev.gyroADC.z = gyroSensor->gyroDev.gyroADCRaw[Z] - gyroSensor->gyroDev.gyroZero[Z];
#endif

        if (gyroSensor->gyroDev.gyroAlign == ALIGN_CUSTOM) {
            alignSensorViaMatrix(&gyroSensor->gyroDev.gyroADC, &gyroSensor->gyroDev.rotationMatrix);
        } else {
            alignSensorViaRotation(&gyroSensor->gyroDev.gyroADC, gyroSensor->gyroDev.gyroAlign);
        }
```

---

## 四、Betaflight中的旋转处理

### 4.1 旋转处理的层次结构

Betaflight中的旋转处理分为**三个层次**：

```
传感器原始数据（芯片坐标系）
    ↓
1. 传感器对齐（Sensor Alignment）
    - gyro_align / acc_align
    - 将传感器坐标系转换为板载坐标系
    ↓
2. 板对齐（Board Alignment）
    - align_board_roll / align_board_pitch / align_board_yaw
    - 将板载坐标系转换为机身坐标系
    ↓
3. 姿态估计旋转（Attitude Estimation）
    - 四元数/旋转矩阵
    - 将机身坐标系转换为地球坐标系
```

### 4.2 第一层：传感器对齐（Sensor Alignment）

**作用**：将IMU芯片的原始坐标系转换为飞控板的坐标系。

**参数**：
- `gyro_align`：陀螺仪对齐方式
- `acc_align`：加速度计对齐方式（通常跟随陀螺仪）

**对齐选项**（`sensor_align_e`）：
```27:43:src/main/common/sensor_alignment.h
typedef enum {
    ALIGN_DEFAULT = 0, // driver-provided alignment

    // the order of these 8 values also correlate to corresponding code in ALIGNMENT_TO_BITMASK.

                            // R, P, Y
    CW0_DEG = 1,            // 00,00,00
    CW90_DEG = 2,           // 00,00,01
    CW180_DEG = 3,          // 00,00,10
    CW270_DEG = 4,          // 00,00,11
    CW0_DEG_FLIP = 5,       // 00,10,00 // _FLIP = 2x90 degree PITCH rotations
    CW90_DEG_FLIP = 6,      // 00,10,01
    CW180_DEG_FLIP = 7,     // 00,10,10
    CW270_DEG_FLIP = 8,     // 00,10,11

    ALIGN_CUSTOM = 9,    // arbitrary sensor angles, e.g. for external sensors
} sensor_align_e;
```

**旋转实现**：
```94:145:src/main/sensors/boardalignment.c
void alignSensorViaRotation(vector3_t *dest, sensor_align_e rotation)
{
    const vector3_t tmp = *dest;

    switch (rotation) {
    default:
    case CW0_DEG:
        dest->x = tmp.x;
        dest->y = tmp.y;
        dest->z = tmp.z;
        break;
    case CW90_DEG:
        dest->x = tmp.y;
        dest->y = -tmp.x;
        dest->z = tmp.z;
        break;
    case CW180_DEG:
        dest->x = -tmp.x;
        dest->y = -tmp.y;
        dest->z = tmp.z;
        break;
    case CW270_DEG:
        dest->x = -tmp.y;
        dest->y = tmp.x;
        dest->z = tmp.z;
        break;
    case CW0_DEG_FLIP:
        dest->x = -tmp.x;
        dest->y = tmp.y;
        dest->z = -tmp.z;
        break;
    case CW90_DEG_FLIP:
        dest->x = tmp.y;
        dest->y = tmp.x;
        dest->z = -tmp.z;
        break;
    case CW180_DEG_FLIP:
        dest->x = tmp.x;
        dest->y = -tmp.y;
        dest->z = -tmp.z;
        break;
    case CW270_DEG_FLIP:
        dest->x = -tmp.y;
        dest->y = -tmp.x;
        dest->z = -tmp.z;
        break;
    }

    if (!standardBoardAlignment) {
        alignBoard(dest);
    }
}
```

**代码位置**：
- 陀螺仪对齐：`src/main/sensors/gyro.c:418-422`
- 加速度计对齐：`src/main/sensors/acceleration.c:44-54`

### 4.3 第二层：板对齐（Board Alignment）

**作用**：当飞控板安装方向与机身坐标系不一致时，进行额外旋转。

**参数**：
- `align_board_roll`：板Roll旋转角度（度）
- `align_board_pitch`：板Pitch旋转角度（度）
- `align_board_yaw`：板Yaw旋转角度（度）

**实现**：
```64:78:src/main/sensors/boardalignment.c
void initBoardAlignment(const boardAlignment_t *boardAlignment)
{
    if (isBoardAlignmentStandard(boardAlignment)) {
        return;
    }

    standardBoardAlignment = false;

    fp_angles_t rotationAngles;
    rotationAngles.angles.roll  = degreesToRadians(boardAlignment->rollDegrees );
    rotationAngles.angles.pitch = degreesToRadians(boardAlignment->pitchDegrees);
    rotationAngles.angles.yaw   = degreesToRadians(boardAlignment->yawDegrees  );

    buildRotationMatrix(&boardRotation, &rotationAngles);
}
```

**旋转矩阵构建**：
```212:232:src/main/common/vector.c
matrix33_t *buildRotationMatrix(matrix33_t *result, const fp_angles_t *rpy)
{
    const float cosx = cos_approx(rpy->angles.roll);
    const float sinx = sin_approx(rpy->angles.roll);
    const float cosy = cos_approx(rpy->angles.pitch);
    const float siny = sin_approx(rpy->angles.pitch);
    const float cosz = cos_approx(rpy->angles.yaw);
    const float sinz = sin_approx(rpy->angles.yaw);

    result->m[0][X] = cosz * cosy;
    result->m[0][Y] = -cosy * sinz;
    result->m[0][Z] = siny;
    result->m[1][X] = sinz * cosx + sinx * cosz * siny;
    result->m[1][Y] = cosz * cosx - sinx * sinz * siny;
    result->m[1][Z] = -sinx * cosy;
    result->m[2][X] = sinx * sinz - cosz * cosx * siny;
    result->m[2][Y] = sinx * cosz + sinz * cosx * siny;
    result->m[2][Z] = cosy * cosx;

    return result;
}
```

**代码位置**：
- 初始化：`src/main/sensors/boardalignment.c:64-78`
- 应用：`src/main/sensors/boardalignment.c:80-83`

### 4.4 第三层：姿态估计旋转（Attitude Estimation）

**作用**：将机身坐标系转换为地球坐标系（NED），计算姿态角。

**核心算法**：Mahony AHRS（互补滤波器）

**实现**：
```212:297:src/main/flight/imu.c
STATIC_UNIT_TESTED void imuMahonyAHRSupdate(float dt,
                                float gx, float gy, float gz,
                                bool useAcc, float ax, float ay, float az,
                                float headingErrMag, float headingErrCog,
                                const float dcmKpGain)
{
    static float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;    // integral error terms scaled by Ki

    // Calculate general spin rate (rad/s)
    const float spin_rate = sqrtf(sq(gx) + sq(gy) + sq(gz));

    float ex = 0, ey = 0, ez = 0;

    // Add error from magnetometer and Cog
    // just rotate input value to body frame
    ex += rMat.m[Z][X] * (headingErrCog + headingErrMag);
    ey += rMat.m[Z][Y] * (headingErrCog + headingErrMag);
    ez += rMat.m[Z][Z] * (headingErrCog + headingErrMag);

    DEBUG_SET(DEBUG_ATTITUDE, 3, (headingErrCog * 100));
    DEBUG_SET(DEBUG_ATTITUDE, 7, lrintf(dcmKpGain * 100.0f));

    // Use measured acceleration vector
    float recipAccNorm = sq(ax) + sq(ay) + sq(az);
    if (useAcc && recipAccNorm > 0.01f) {
        // Normalise accelerometer measurement; useAcc is true when all smoothed acc axes are within 20% of 1G
        recipAccNorm = invSqrt(recipAccNorm);

        ax *= recipAccNorm;
        ay *= recipAccNorm;
        az *= recipAccNorm;

        // Error is sum of cross product between estimated direction and measured direction of gravity
        ex += (ay * rMat.m[2][2] - az * rMat.m[2][1]);
        ey += (az * rMat.m[2][0] - ax * rMat.m[2][2]);
        ez += (ax * rMat.m[2][1] - ay * rMat.m[2][0]);
    }

    // Compute and apply integral feedback if enabled
    if (imuRuntimeConfig.imuDcmKi > 0.0f) {
        // Stop integrating if spinning beyond the certain limit
        if (spin_rate < DEGREES_TO_RADIANS(SPIN_RATE_LIMIT)) {
            const float dcmKiGain = imuRuntimeConfig.imuDcmKi;
            integralFBx += dcmKiGain * ex * dt;    // integral error scaled by Ki
            integralFBy += dcmKiGain * ey * dt;
            integralFBz += dcmKiGain * ez * dt;
        }
    } else {
        integralFBx = 0.0f;    // prevent integral windup
        integralFBy = 0.0f;
        integralFBz = 0.0f;
    }

    // Apply proportional and integral feedback
    gx += dcmKpGain * ex + integralFBx;
    gy += dcmKpGain * ey + integralFBy;
    gz += dcmKpGain * ez + integralFBz;

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);

    quaternion_t buffer;
    buffer.w = q.w;
    buffer.x = q.x;
    buffer.y = q.y;
    buffer.z = q.z;

    q.w += (-buffer.x * gx - buffer.y * gy - buffer.z * gz);
    q.x += (+buffer.w * gx + buffer.y * gz - buffer.z * gy);
    q.y += (+buffer.w * gy - buffer.x * gz + buffer.z * gx);
    q.z += (+buffer.w * gz + buffer.x * gy - buffer.y * gx);

    // Normalise quaternion
    float recipNorm = invSqrt(sq(q.w) + sq(q.x) + sq(q.y) + sq(q.z));
    q.w *= recipNorm;
    q.x *= recipNorm;
    q.y *= recipNorm;
    q.z *= recipNorm;

    // Pre-compute rotation matrix from quaternion
    imuComputeRotationMatrix();

    attitudeIsEstablished = true;
}
```

**旋转矩阵计算**：
```145:165:src/main/flight/imu.c
STATIC_UNIT_TESTED void imuComputeRotationMatrix(void)
{
    imuQuaternionComputeProducts(&q, &qP);

    rMat.m[0][0] = 1.0f - 2.0f * qP.yy - 2.0f * qP.zz;
    rMat.m[0][1] = 2.0f * (qP.xy + -qP.wz);
    rMat.m[0][2] = 2.0f * (qP.xz - -qP.wy);

    rMat.m[1][0] = 2.0f * (qP.xy - -qP.wz);
    rMat.m[1][1] = 1.0f - 2.0f * qP.xx - 2.0f * qP.zz;
    rMat.m[1][2] = 2.0f * (qP.yz + -qP.wx);

    rMat.m[2][0] = 2.0f * (qP.xz + -qP.wy);
    rMat.m[2][1] = 2.0f * (qP.yz - -qP.wx);
    rMat.m[2][2] = 1.0f - 2.0f * qP.xx - 2.0f * qP.yy;

#if defined(SIMULATOR_BUILD) && !defined(USE_IMU_CALC) && !defined(SET_IMU_FROM_EULER)
    rMat.m[1][0] = -2.0f * (qP.xy - -qP.wz);
    rMat.m[2][0] = -2.0f * (qP.xz + -qP.wz);
#endif
}
```

**欧拉角计算**：
```299:320:src/main/flight/imu.c
STATIC_UNIT_TESTED void imuUpdateEulerAngles(void)
{
    quaternionProducts buffer;

    if (FLIGHT_MODE(HEADFREE_MODE)) {
       imuQuaternionComputeProducts(&headfree, &buffer);

       attitude.values.roll = lrintf(atan2_approx((+2.0f * (buffer.wx + buffer.yz)), (+1.0f - 2.0f * (buffer.xx + buffer.yy))) * (1800.0f / M_PIf));
       attitude.values.pitch = lrintf(((0.5f * M_PIf) - acos_approx(+2.0f * (buffer.wy - buffer.xz))) * (1800.0f / M_PIf));
       attitude.values.yaw = lrintf((-atan2_approx((+2.0f * (buffer.wz + buffer.xy)), (+1.0f - 2.0f * (buffer.yy + buffer.zz))) * (1800.0f / M_PIf)));
       imuAttitudeQuaternion = headfree;
    } else {
       attitude.values.roll = lrintf(atan2_approx(rMat.m[2][1], rMat.m[2][2]) * (1800.0f / M_PIf));
       attitude.values.pitch = lrintf(((0.5f * M_PIf) - acos_approx(-rMat.m[2][0])) * (1800.0f / M_PIf));
       attitude.values.yaw = lrintf((-atan2_approx(rMat.m[1][0], rMat.m[0][0]) * (1800.0f / M_PIf)));
       imuAttitudeQuaternion = q; //using current q quaternion  for blackbox log
    }

    if (attitude.values.yaw < 0) {
        attitude.values.yaw += 3600;
    }
}
```

**代码位置**：
- 姿态更新入口：`src/main/flight/imu.c:743`
- Mahony算法：`src/main/flight/imu.c:212-297`
- 旋转矩阵：`src/main/flight/imu.c:145-165`
- 欧拉角：`src/main/flight/imu.c:299-320`

---

## 五、控制IMU方向和机身方向的参数

### 5.1 传感器对齐参数

#### `gyro_align` / `acc_align`

**作用**：指定传感器相对于飞控板的对齐方式。

**可选值**：
- `CW0_DEG` (1)：0度，无旋转
- `CW90_DEG` (2)：顺时针90度
- `CW180_DEG` (3)：180度
- `CW270_DEG` (4)：顺时针270度
- `CW0_DEG_FLIP` (5)：翻转（180度Pitch）
- `CW90_DEG_FLIP` (6)：翻转+90度
- `CW180_DEG_FLIP` (7)：翻转+180度
- `CW270_DEG_FLIP` (8)：翻转+270度
- `ALIGN_CUSTOM` (9)：自定义对齐（使用`gyro_1_align_roll/pitch/yaw`）

**配置位置**：
- CLI命令：`set gyro_align = CW0_DEG`
- 代码定义：`src/main/pg/gyrodev.c`（编译时定义）

**影响**：
- 直接影响陀螺仪和加速度计的读数方向
- 错误设置会导致姿态估计完全错误

**示例**：
- 如果IMU芯片旋转了90度，设置`gyro_align = CW90_DEG`
- 如果IMU芯片翻转安装，设置`gyro_align = CW0_DEG_FLIP`

### 5.2 板对齐参数

#### `align_board_roll` / `align_board_pitch` / `align_board_yaw`

**作用**：当飞控板安装方向与机身坐标系不一致时，进行额外旋转。

**单位**：度（degrees）
**范围**：-180 到 360

**配置位置**：
```997:1000:src/main/cli/settings.c
    { "align_board_roll",           VAR_INT16  | MASTER_VALUE, .config.minmax = { -180, 360 }, PG_BOARD_ALIGNMENT, offsetof(boardAlignment_t, rollDegrees) },
    { "align_board_pitch",          VAR_INT16  | MASTER_VALUE, .config.minmax = { -180, 360 }, PG_BOARD_ALIGNMENT, offsetof(boardAlignment_t, pitchDegrees) },
    { "align_board_yaw",            VAR_INT16  | MASTER_VALUE, .config.minmax = { -180, 360 }, PG_BOARD_ALIGNMENT, offsetof(boardAlignment_t, yawDegrees) },
```

**影响**：
- 在所有传感器对齐之后应用
- 影响所有传感器（陀螺仪、加速度计、磁力计）
- 用于补偿飞控板安装角度偏差

**示例**：
- 飞控板向前倾斜10度：`set align_board_pitch = 10`
- 飞控板向右倾斜5度：`set align_board_roll = -5`
- 飞控板旋转45度：`set align_board_yaw = 45`

### 5.3 自定义对齐参数

#### `gyro_1_align_roll` / `gyro_1_align_pitch` / `gyro_1_align_yaw`

**作用**：当`gyro_align = ALIGN_CUSTOM`时，指定精确的旋转角度。

**单位**：0.1度（decidegrees），例如90度 = 900

**配置位置**：
```106:110:src/main/pg/gyrodev.c
#define GYRO_1_CUSTOM_ALIGN     SENSOR_ALIGNMENT( GYRO_1_ALIGN_ROLL / 10, GYRO_1_ALIGN_PITCH / 10, GYRO_1_ALIGN_YAW / 10 )
```

**影响**：
- 允许任意角度的旋转（不限于90度的倍数）
- 用于特殊安装情况

**示例**：
- 旋转45度：`#define GYRO_1_ALIGN_YAW 450`
- 倾斜30度：`#define GYRO_1_ALIGN_PITCH 300`

### 5.4 参数应用顺序和规则

**应用顺序**：
1. **传感器对齐**（`gyro_align` / `acc_align`）
   - 首先应用，将传感器坐标系转换为板坐标系
2. **板对齐**（`align_board_*`）
   - 然后应用，将板坐标系转换为机身坐标系
3. **姿态估计**
   - 最后使用，将机身坐标系转换为地球坐标系

**规则**：
- `gyro_align`和`acc_align`通常设置为相同值（因为IMU是组合芯片）
- `align_board_*`参数在所有传感器上统一应用
- 参数设置错误会导致：
  - 姿态估计错误
  - 控制响应反向
  - 飞行不稳定

**调试方法**：
1. 水平放置飞行器，检查姿态角是否接近0
2. 缓慢旋转各轴，检查姿态角变化方向是否正确
3. 使用`status`命令查看传感器读数

---

## 六、所有旋转处理代码位置总结

### 6.1 传感器对齐

| 功能 | 文件位置 | 函数/代码 |
|------|----------|-----------|
| 陀螺仪对齐 | `src/main/sensors/gyro.c:418-422` | `alignSensorViaRotation()` / `alignSensorViaMatrix()` |
| 加速度计对齐 | `src/main/sensors/acceleration.c:44-54` | `alignAccelerometer()` |
| 磁力计对齐 | `src/main/sensors/compass.c` | `alignSensorViaRotation()` |
| 对齐旋转实现 | `src/main/sensors/boardalignment.c:94-145` | `alignSensorViaRotation()` |
| 对齐矩阵实现 | `src/main/sensors/boardalignment.c:85-92` | `alignSensorViaMatrix()` |
| 对齐枚举定义 | `src/main/common/sensor_alignment.h:27-43` | `sensor_align_e` |

### 6.2 板对齐

| 功能 | 文件位置 | 函数/代码 |
|------|----------|-----------|
| 板对齐初始化 | `src/main/sensors/boardalignment.c:64-78` | `initBoardAlignment()` |
| 板对齐应用 | `src/main/sensors/boardalignment.c:80-83` | `alignBoard()` |
| 旋转矩阵构建 | `src/main/common/vector.c:212-232` | `buildRotationMatrix()` |
| 旋转矩阵应用 | `src/main/common/vector.c:234-237` | `applyRotationMatrix()` |
| 板对齐参数定义 | `src/main/sensors/boardalignment.h:29-33` | `boardAlignment_t` |

### 6.3 姿态估计旋转

| 功能 | 文件位置 | 函数/代码 |
|------|----------|-----------|
| 姿态更新入口 | `src/main/flight/imu.c:743` | `imuUpdateAttitude()` |
| 姿态计算主函数 | `src/main/flight/imu.c:650-723` | `imuCalculateEstimatedAttitude()` |
| Mahony AHRS算法 | `src/main/flight/imu.c:212-297` | `imuMahonyAHRSupdate()` |
| 四元数更新 | `src/main/flight/imu.c:275-291` | 四元数积分 |
| 旋转矩阵计算 | `src/main/flight/imu.c:145-165` | `imuComputeRotationMatrix()` |
| 欧拉角计算 | `src/main/flight/imu.c:299-320` | `imuUpdateEulerAngles()` |
| 四元数乘积 | `src/main/flight/imu.c:131-143` | `imuQuaternionComputeProducts()` |

### 6.4 参数配置

| 功能 | 文件位置 | 说明 |
|------|----------|------|
| 陀螺仪对齐配置 | `src/main/pg/gyrodev.c` | `GYRO_1_ALIGN`等宏定义 |
| 板对齐CLI命令 | `src/main/cli/settings.c:997-1000` | CLI参数定义 |
| 板对齐MSP协议 | `src/main/msp/msp.c:1350-1353, 3725-3728` | MSP协议支持 |

---

## 七、实际应用示例

### 示例1：标准安装（无旋转）

**情况**：IMU芯片标准安装，飞控板标准安装

**参数设置**：
```
gyro_align = CW0_DEG
acc_align = CW0_DEG
align_board_roll = 0
align_board_pitch = 0
align_board_yaw = 0
```

**结果**：传感器读数直接对应机身坐标系

### 示例2：IMU芯片旋转90度

**情况**：IMU芯片顺时针旋转90度安装

**参数设置**：
```
gyro_align = CW90_DEG
acc_align = CW90_DEG
align_board_roll = 0
align_board_pitch = 0
align_board_yaw = 0
```

**结果**：传感器X轴对应机身Y轴，Y轴对应机身-X轴

### 示例3：飞控板倾斜安装

**情况**：飞控板向前倾斜10度安装

**参数设置**：
```
gyro_align = CW0_DEG
acc_align = CW0_DEG
align_board_roll = 0
align_board_pitch = 10
align_board_yaw = 0
```

**结果**：补偿10度倾斜，姿态估计正确

### 示例4：IMU翻转+板旋转

**情况**：IMU芯片翻转安装，飞控板旋转45度

**参数设置**：
```
gyro_align = CW0_DEG_FLIP
acc_align = CW0_DEG_FLIP
align_board_roll = 0
align_board_pitch = 0
align_board_yaw = 45
```

**结果**：先翻转传感器，再旋转45度

---

## 八、调试和验证

### 8.1 验证传感器对齐

1. **水平静止测试**：
   - 水平放置飞行器
   - 检查姿态角：Roll≈0, Pitch≈0
   - 检查加速度计：Z轴≈1G，X/Y轴≈0

2. **旋转测试**：
   - 缓慢旋转各轴
   - 检查姿态角变化方向是否正确
   - 检查陀螺仪读数方向是否正确

### 8.2 常见问题

**问题1：姿态角反向**
- 原因：`gyro_align`设置错误
- 解决：尝试其他对齐选项

**问题2：姿态角倾斜**
- 原因：`align_board_*`参数不正确
- 解决：调整板对齐参数

**问题3：控制响应反向**
- 原因：传感器对齐或板对齐错误
- 解决：检查所有对齐参数

---

## 总结

Betaflight中的IMU和姿态旋转处理是一个**三层旋转系统**：

1. **传感器对齐层**：将传感器坐标系转换为板坐标系
2. **板对齐层**：将板坐标系转换为机身坐标系
3. **姿态估计层**：将机身坐标系转换为地球坐标系

每一层都有对应的参数和代码实现，正确配置这些参数对于飞行器的稳定控制至关重要。

