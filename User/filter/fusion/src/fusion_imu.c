/*
 * Application adapter for the xioTechnologies Fusion implementation.
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Techno Road Inc.
 */

#include "fusion_imu.h"
#include "FusionAhrs.h"
#include "FusionBias.h"

#define DEFAULT_SAMPLE_RATE_HZ (100.0f)
#define FUSION_GAIN (0.5f)
#define ACCELERATION_REJECTION_DEG (10.0f)
#define REJECTION_TIMEOUT_SEC (5.0f)
#define STATIONARY_THRESHOLD_DPS (1.0f)
#define STATIONARY_SAMPLE_COUNT (1000.0f)
#define RADIANS_TO_DEGREES (57.29577951308232f)

static FusionAhrs fusionAhrs;
static FusionBias fusionBias;

static FusionVector CorrectGyroscope(const float gx, const float gy,
    const float gz);

void FusionImu_Initialize(float samplePeriod) {
  const float sampleRate = samplePeriod > 0.0f ?
      1.0f / samplePeriod : DEFAULT_SAMPLE_RATE_HZ;

  FusionAhrsInitialise(&fusionAhrs);
  FusionAhrsSettings ahrsSettings = fusionAhrsDefaultSettings;
  ahrsSettings.sampleRate = sampleRate;
  ahrsSettings.convention = FusionConventionNwu;
  ahrsSettings.gain = FUSION_GAIN;
  ahrsSettings.gyroscopeRange = 0.0f;
  ahrsSettings.accelerationRejection = ACCELERATION_REJECTION_DEG;
  ahrsSettings.magneticRejection = 0.0f;
  ahrsSettings.rejectionTimeout = REJECTION_TIMEOUT_SEC;
  FusionAhrsSetSettings(&fusionAhrs, &ahrsSettings);

  FusionBiasInitialise(&fusionBias);
  FusionBiasSettings biasSettings = fusionBiasDefaultSettings;
  biasSettings.sampleRate = sampleRate;
  biasSettings.stationaryThreshold = STATIONARY_THRESHOLD_DPS;
  biasSettings.stationaryPeriod = STATIONARY_SAMPLE_COUNT / sampleRate;
  FusionBiasSetSettings(&fusionBias, &biasSettings);
}

void FusionImu_ResetOrientation(void) {
  FusionAhrsRestart(&fusionAhrs);
}

void FusionImu_Update6Axis(float gx, float gy, float gz,
    float ax, float ay, float az) {
  const FusionVector gyroscope = CorrectGyroscope(gx, gy, gz);
  const FusionVector accelerometer = { .axis = { ax, ay, az } };

  FusionAhrsUpdateNoMagnetometer(&fusionAhrs, gyroscope, accelerometer);
}

void FusionImu_UpdateGyroscope(float gx, float gy, float gz) {
  const FusionVector gyroscope = CorrectGyroscope(gx, gy, gz);

  FusionAhrsUpdate(&fusionAhrs, gyroscope, FUSION_VECTOR_ZERO,
      FUSION_VECTOR_ZERO);
}

void FusionImu_GetQuaternion(float out[4]) {
  const FusionQuaternion quaternion = FusionAhrsGetQuaternion(&fusionAhrs);

  out[0] = quaternion.element.w;
  out[1] = quaternion.element.x;
  out[2] = quaternion.element.y;
  out[3] = quaternion.element.z;
}

int FusionImu_IsRest(void) {
  return fusionBias.timer >= fusionBias.timeout;
}

static FusionVector CorrectGyroscope(const float gx, const float gy,
    const float gz) {
  const FusionVector gyroscope = {
      .axis = {
          gx * RADIANS_TO_DEGREES,
          gy * RADIANS_TO_DEGREES,
          gz * RADIANS_TO_DEGREES,
      }
  };
  return FusionBiasUpdate(&fusionBias, gyroscope);
}
