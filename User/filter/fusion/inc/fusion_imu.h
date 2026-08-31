/*
 * Application adapter for the xioTechnologies Fusion implementation.
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Techno Road Inc.
 */

#ifndef FUSION_IMU_H
#define FUSION_IMU_H

void FusionImu_Initialize(float samplePeriod);
void FusionImu_ResetOrientation(void);
void FusionImu_Update6Axis(float gx, float gy, float gz,
    float ax, float ay, float az);
void FusionImu_UpdateGyroscope(float gx, float gy, float gz);
void FusionImu_GetQuaternion(float out[4]);
int FusionImu_IsRest(void);

#endif /* FUSION_IMU_H */
