#ifndef __CONTROL_H
#define __CONTROL_H

#include "filter.h"

extern unsigned int g_nMainEventCount;
extern unsigned int g_nGetPulseCount;
extern unsigned int g_nSpeedControlCount;
extern int g_nSpeedControlPeriod;
extern float g_fCarAngle;
extern unsigned int g_nLeftMotorPulse;
extern int g_nSpeedTarget;
extern float g_fLeftMotorOut, g_fRightMotorOut;
extern float CAR_SPEED_SET;
extern float CAR_LEFT_SPEED_SET, CAR_RIGHT_SPEED_SET;
extern float CAR_DISTANCE_SET;

void Move(float distance);
void Turn(float angle);
void GetMpuData(void);
void AngleCalculate(void);
void GetMotorPulse(void);
void SpeedInnerControl(void);
void SetMotorVoltageAndDirection(int nLeftMotorPwm,int nRightMotorPwm);
void MotorOutput(void);
void AngleControl(void);
void SpeedControl(void);
void SpeedControlOutput(void);
void BalanceControl(void);

#endif