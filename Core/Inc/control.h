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
extern int g_nLeftMotorOutput;

void GetMpuData(void);
void AngleCalculate(void);
void GetMotorPulse(void);
int SpeedInnerControl(int nPulse, int nTarget, int nPwm, int nErrorPrev);
void SetMotorVoltageAndDirection(int nLeftMotorPwm,int nRightMotorPwm);
void MotorOutput(void);
void AngleControl(void);
void SpeedControl(void);
void SpeedControlOutput(void);

#endif