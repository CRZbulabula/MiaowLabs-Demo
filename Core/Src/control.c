#include "control.h"
#include "filter.h"
#include "mpu6050.h"
#include "math.h"
#include "outputdata.h"
#include "tim.h"
#include "main.h"

#define MOTOR_OUT_DEAD_VAL 0 // Dead band value
#define MOTOR_OUT_MAX 1000 // Maximum duty cycle
#define MOTOR_OUT_MIN (-1000) // Minimum duty cycle

#define CAR_ANGLE_SET 0 // Target angle
#define CAR_ANGLE_SPEED_SET 0 // Target angular velocity

short x_nAcc, y_nAcc, z_nAcc; // Acceleration data, X, Y, Z
short x_nGyro, y_nGyro, z_nGyro; // Gyroscope data, X, Y, Z
float x_fAcc, y_fAcc, z_fAcc;

float g_fAccAngle;
float g_fGyroAngleSpeed; // Angular velocity on X
float g_fCarAngle; // Car angle
float dt = 0.005;

unsigned int g_nMainEventCount;
unsigned int g_nGetPulseCount;

unsigned int g_nLeftMotorPulse, g_nRightMotorPulse;

// For speed inner control
int nPwmBais;
int nLeftMotorPwm, nRightMotorPwm;
int nLeftMotorErrorPrev, nRightMotorErrorPrev;

float g_fLeftMotorOut, g_fRightMotorOut;
float g_fAngleControlOut;

float CAR_SPEED_SET = 0; // Target speed
float CAR_DISTANCE_SET = 0; // Target distance
float CAR_TURN_SET = 0; // Target turn angle

#define CAR_POSITION_MAX 900
#define CAR_POSITION_MIN (-900)
#define SPEED_CONTROL_PERIOD 25

unsigned int g_nSpeedControlCount;
int g_nSpeedControlPeriod;

// For speed control
float g_fCarSpeed;
float g_fCarSpeedPrev;
float g_fCarPosition;
long g_lLeftMotorPulseSigma;
long g_lRightMotorPulseSigma;
float g_fSpeedControlOut, g_fSpeedControlOutNew, g_fSpeedControlOutOld;

// For speed outer control
#define MOVE_START_SPEED 50
#define MOVE_MAX_SPEED 30

int MOVE_CONTROL = 0;
int TURN_CONTROL = 0;

float CAR_LEFT_SPEED_SET = 0;
float CAR_RIGHT_SPEED_SET = 0;

float g_fCarLeftSpeed, g_fCarRightSpeed;
float g_fCarLeftSpeedPrev, g_fCarRightSpeedPrev;
float g_fCarLeftPosition, g_fCarRightPosition;
float g_fLeftSpeedControlOut, g_fLeftSpeedControlOutNew, g_fLeftSpeedControlOutOld;
float g_fRightSpeedControlOut, g_fRightSpeedControlOutNew, g_fRightSpeedControlOutOld;

#define CAR_ZERO_ANGLE (g_fCarAngleOffset)

// Outer control Mode
float g_fCarAngleOffset = 5.0;
// Inner control Mode
// float g_fCarAngleOffset = -2.5;

void Move(float distance)
{
	CAR_DISTANCE_SET = distance;
	if (CAR_DISTANCE_SET > 0) {
		CAR_LEFT_SPEED_SET = CAR_RIGHT_SPEED_SET = MOVE_START_SPEED;
	} else {
		CAR_LEFT_SPEED_SET = CAR_RIGHT_SPEED_SET = -MOVE_START_SPEED;
	}
	MOVE_CONTROL = 1;
	while (fabs(CAR_RIGHT_SPEED_SET) > 0.05) {
		//HAL_Delay(100);
		HAL_Delay(500);
		printf("%.4f %.4f %.4f\n", CAR_DISTANCE_SET, CAR_LEFT_SPEED_SET, g_fCarLeftSpeed);
		continue;
	}
	MOVE_CONTROL = 0;
}

void Turn(float angle)
{
	CAR_TURN_SET = angle / 2.5;
	if (fabs(CAR_TURN_SET) > 0) {
		CAR_LEFT_SPEED_SET = CAR_RIGHT_SPEED_SET = MOVE_START_SPEED;
	}
	TURN_CONTROL = 1;
	while (fabs(CAR_LEFT_SPEED_SET) > 0.05) {
		HAL_Delay(100);
		//HAL_Delay(500);
		//printf("%.4f %.4f %.4f\n", CAR_TURN_SET, CAR_LEFT_SPEED_SET, g_fCarLeftSpeed);
		continue;
	}
	TURN_CONTROL = 0;
}

void GetMotorPulse(void)
{
	g_nRightMotorPulse = (short) (__HAL_TIM_GET_COUNTER(&htim4));
	g_nRightMotorPulse = (-g_nRightMotorPulse);
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	g_nLeftMotorPulse = (short) (__HAL_TIM_GET_COUNTER(&htim2));
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	g_lLeftMotorPulseSigma += g_nLeftMotorPulse;
	g_lRightMotorPulseSigma += g_nRightMotorPulse;
}

void GetMpuData(void)
{
	MPU_Get_Accelerometer(&x_nAcc, &y_nAcc, &z_nAcc);
	MPU_Get_Gyroscope(&x_nGyro, &y_nGyro, &z_nGyro);
}

void AngleCalculate(void)
{
	x_fAcc = x_nAcc / 16384.0;
	y_fAcc = y_nAcc / 16384.0;
	z_fAcc = z_nAcc / 16384.0;
	
	g_fAccAngle = atan2(y_fAcc, z_fAcc) * 180.0 / 3.14;
	
	g_fGyroAngleSpeed = x_nGyro / 16.4;
	
	g_fCarAngle = ComplementaryFilter(g_fAccAngle, g_fGyroAngleSpeed, dt);

	g_fCarAngle = g_fCarAngle - CAR_ZERO_ANGLE;

	//OutData[0] = g_fAccAngle;
	//OutData[1] = g_fGyroAngleSpeed;
	//OutData[2] = g_fCarAngle;
}

void AngleControl(void)
{
	float fP = 65.0;
	float fD = 2.3;
	g_fAngleControlOut = (CAR_ANGLE_SET - g_fCarAngle) * fP + 
						 (CAR_ANGLE_SPEED_SET - g_fGyroAngleSpeed) * fD;
}

void MotorOutput(void)
{
	// Outer control Mode
	g_fLeftMotorOut = g_fAngleControlOut - g_fLeftSpeedControlOut;
	g_fRightMotorOut = g_fAngleControlOut - g_fRightSpeedControlOut;
	
	// Inner control Mode
	//g_fLeftMotorOut = g_fAngleControlOut - nLeftMotorPwm;
	//g_fRightMotorOut = g_fAngleControlOut - nRightMotorPwm;

	if ((int) g_fLeftMotorOut > 0) g_fLeftMotorOut += MOTOR_OUT_DEAD_VAL;
	else if((int) g_fLeftMotorOut < 0) g_fLeftMotorOut -= MOTOR_OUT_DEAD_VAL;
	if ((int) g_fRightMotorOut > 0) g_fRightMotorOut += MOTOR_OUT_DEAD_VAL;
	else if((int) g_fRightMotorOut < 0) g_fRightMotorOut -= MOTOR_OUT_DEAD_VAL;
	
	if ((int) g_fLeftMotorOut > MOTOR_OUT_MAX) g_fLeftMotorOut = MOTOR_OUT_MAX;
	if ((int) g_fLeftMotorOut < MOTOR_OUT_MIN) g_fLeftMotorOut = MOTOR_OUT_MIN;
	if ((int) g_fRightMotorOut > MOTOR_OUT_MAX) g_fRightMotorOut = MOTOR_OUT_MAX;
	if ((int) g_fRightMotorOut < MOTOR_OUT_MIN) g_fRightMotorOut = MOTOR_OUT_MIN;

	SetMotorVoltageAndDirection((int) g_fLeftMotorOut, (int) g_fRightMotorOut);
}

void SetMotorVoltageAndDirection(int LeftMotorPwm, int RightMotorPwm)
{
	if(RightMotorPwm < 0)
	{
		HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
		RightMotorPwm = (-RightMotorPwm);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, RightMotorPwm);
	} else {
		HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, RightMotorPwm);
	}
	
	if(LeftMotorPwm < 0)
	{
		HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
		LeftMotorPwm = (-LeftMotorPwm);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, LeftMotorPwm);
	} else {
		HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, LeftMotorPwm);
	}
}

void SpeedInnerControl(void)
{
	float fP = 10.0, fI = 0.9;

	float CarSpeedPrev = (g_nLeftMotorPulse + g_nRightMotorPulse) / 2;
	if (fabs(CAR_DISTANCE_SET) > 1) {
		if (fabs(CarSpeedPrev * 0.025) > fabs(CAR_DISTANCE_SET)) {
			CAR_DISTANCE_SET = 0;
		} else {
			CAR_DISTANCE_SET -= CarSpeedPrev * dt;
		}
		if (CAR_DISTANCE_SET > 0) {
			CAR_SPEED_SET = 30;
		} else {
			CAR_SPEED_SET = -30;
		}
	} else {
		CAR_SPEED_SET = 0;
	}

	int nLeftError = CAR_SPEED_SET - g_nLeftMotorPulse;
	int nRightError = CAR_SPEED_SET - g_nRightMotorPulse;

	float nLeftPwmBais = fP * (nLeftError - nLeftMotorErrorPrev) + fI * nLeftError;
	float nRightPwmBais = fP * (nRightError - nRightMotorErrorPrev) + fI * nRightError;

	nLeftMotorErrorPrev = nLeftError;
	nRightMotorErrorPrev = nRightError;

	nLeftMotorPwm += nLeftPwmBais;
	nRightMotorPwm += nRightPwmBais;
}

void SpeedControl(void)
{
	float fP = 10.25, fI = 0.108;
	float fDelta;
	
	g_fCarSpeed = (g_lLeftMotorPulseSigma + g_lRightMotorPulseSigma ) / 2;
	g_lLeftMotorPulseSigma = g_lRightMotorPulseSigma = 0;
	g_fCarSpeed = 0.7 * g_fCarSpeedPrev + 0.3 * g_fCarSpeed;
	g_fCarSpeedPrev = g_fCarSpeed;
	
	if (fabs(CAR_DISTANCE_SET) > 1) {
		if (fabs(g_fCarSpeedPrev * 0.025) > fabs(CAR_DISTANCE_SET)) {
			CAR_DISTANCE_SET = 0;
		} else {
			CAR_DISTANCE_SET -= g_fCarSpeedPrev * 0.025;
		}
		if (CAR_DISTANCE_SET > 0) {
			CAR_SPEED_SET = 30;
		} else {
			CAR_SPEED_SET = -30;
		}
	} else {
		CAR_SPEED_SET = 0;
	}
	
	fDelta = CAR_SPEED_SET - g_fCarSpeed;
	g_fCarPosition += fDelta;
	
	if((int) g_fCarPosition > CAR_POSITION_MAX) g_fCarPosition = CAR_POSITION_MAX;
	if((int) g_fCarPosition < CAR_POSITION_MIN) g_fCarPosition = CAR_POSITION_MIN;
	
	g_fSpeedControlOutOld = g_fSpeedControlOutNew;
	g_fSpeedControlOutNew = fDelta * fP + g_fCarPosition * fI;
}

void SpeedControlOutput(void)
{
	float fValue;
	fValue = g_fSpeedControlOutNew - g_fSpeedControlOutOld ;
	g_fSpeedControlOut = fValue * (g_nSpeedControlPeriod + 1) / 
						 SPEED_CONTROL_PERIOD + g_fSpeedControlOutOld;
}

void SpeedOuterControl(void)
{
	float fP = 10.25, fI = 0.108;
	float fLeftDelta, fRightDelta;
	
	g_fCarLeftSpeed = g_lLeftMotorPulseSigma;
	g_fCarLeftSpeed = 0.7 * g_fCarLeftSpeedPrev + 0.3 * g_fCarLeftSpeed;
	g_fCarLeftSpeedPrev = g_fCarLeftSpeed;
	g_fCarRightSpeed = g_lRightMotorPulseSigma;
	g_fCarRightSpeed = 0.7 * g_fCarRightSpeedPrev + 0.3 * g_fCarRightSpeed;
	g_fCarRightSpeedPrev = g_fCarRightSpeed;
	g_lLeftMotorPulseSigma = g_lRightMotorPulseSigma = 0;
	
	// Move Control
	if (fabs(CAR_DISTANCE_SET) > 1) {
		g_fCarSpeedPrev = (g_fCarLeftSpeedPrev + g_fCarRightSpeedPrev) / 2.0;
		if (fabs(g_fCarSpeedPrev * 0.025) > fabs(CAR_DISTANCE_SET)) {
			CAR_DISTANCE_SET = 0;
		} else {
			CAR_DISTANCE_SET -= g_fCarSpeedPrev * 0.025;
		}
		if (fabs(g_fCarSpeedPrev) > MOVE_MAX_SPEED) {
			if (CAR_LEFT_SPEED_SET > MOVE_MAX_SPEED) {
				CAR_LEFT_SPEED_SET = CAR_RIGHT_SPEED_SET = MOVE_MAX_SPEED;
			} else if (CAR_LEFT_SPEED_SET < -MOVE_MAX_SPEED) {
				CAR_LEFT_SPEED_SET = CAR_RIGHT_SPEED_SET = -MOVE_MAX_SPEED;
			}
		}
	} else if (MOVE_CONTROL) {
		if (CAR_LEFT_SPEED_SET > 0) {
			CAR_LEFT_SPEED_SET -= 0.25;
			CAR_RIGHT_SPEED_SET -= 0.25;
		} else if (CAR_LEFT_SPEED_SET < 0) {
			CAR_LEFT_SPEED_SET += 0.75;
			CAR_RIGHT_SPEED_SET += 0.75;
		}
	}

	// Turn Control
	if (fabs(CAR_TURN_SET) > 1) {
		float g_fCarSpeedDeltaPrev = (g_fCarLeftSpeedPrev - g_fCarRightSpeedPrev);
		if (fabs(g_fCarSpeedDeltaPrev * 0.025) > fabs(CAR_TURN_SET)) {
			CAR_TURN_SET = 0;
		} else {
			CAR_TURN_SET -= g_fCarSpeedDeltaPrev * 0.025;
		}

		if (fabs(g_fCarLeftSpeedPrev) > MOVE_MAX_SPEED) {
			if (CAR_TURN_SET > 0) {
				CAR_LEFT_SPEED_SET = 25;
			} else {
				CAR_LEFT_SPEED_SET = 15;
			}
		}
		if (fabs(g_fCarRightSpeedPrev) > MOVE_MAX_SPEED) {
			if (CAR_TURN_SET > 0) {
				CAR_RIGHT_SPEED_SET = 15;
			} else {
				CAR_RIGHT_SPEED_SET = 25;
			}
		}
	} else if (TURN_CONTROL) {
		if (CAR_LEFT_SPEED_SET > 0) {
			CAR_LEFT_SPEED_SET -= 0.25;
		}
		if (CAR_RIGHT_SPEED_SET > 0) {
			CAR_RIGHT_SPEED_SET -= 0.25;
		}
	}

	fLeftDelta = CAR_LEFT_SPEED_SET - g_fCarLeftSpeed;
	g_fCarLeftPosition += fLeftDelta;
	fRightDelta = CAR_RIGHT_SPEED_SET - g_fCarRightSpeed;
	g_fCarRightPosition += fRightDelta;
	
	if((int) g_fCarLeftPosition > CAR_POSITION_MAX) g_fCarLeftPosition = CAR_POSITION_MAX;
	if((int) g_fCarLeftPosition < CAR_POSITION_MIN) g_fCarLeftPosition = CAR_POSITION_MIN;
	if((int) g_fCarRightPosition > CAR_POSITION_MAX) g_fCarRightPosition = CAR_POSITION_MAX;
	if((int) g_fCarRightPosition < CAR_POSITION_MIN) g_fCarRightPosition = CAR_POSITION_MIN;
	
	g_fLeftSpeedControlOutOld = g_fLeftSpeedControlOutNew;
	g_fLeftSpeedControlOutNew = fLeftDelta * fP + g_fCarLeftPosition * fI;
	g_fRightSpeedControlOutOld = g_fRightSpeedControlOutNew;
	g_fRightSpeedControlOutNew = fRightDelta * fP + g_fCarRightPosition * fI;
}

void SpeedOuterControlOutput(void)
{
	float fLeftValue;
	fLeftValue = g_fLeftSpeedControlOutNew - g_fLeftSpeedControlOutOld;
	g_fLeftSpeedControlOut = fLeftValue * (g_nSpeedControlPeriod + 1) / 
						 SPEED_CONTROL_PERIOD + g_fLeftSpeedControlOutOld;
	float fRightValue;
	fRightValue = g_fRightSpeedControlOutNew - g_fRightSpeedControlOutOld;
	g_fRightSpeedControlOut = fRightValue * (g_nSpeedControlPeriod + 1) / 
						 SPEED_CONTROL_PERIOD + g_fRightSpeedControlOutOld;
}

void BalanceControl(void)
{
	g_nMainEventCount++;

	g_nSpeedControlPeriod++;
	//SpeedControlOutput();
	SpeedOuterControlOutput();

	if (g_nMainEventCount >= 5) {
		// Reset and capture pulses every 5ms
		g_nMainEventCount = 0;
		GetMotorPulse();
	} else if (g_nMainEventCount == 1) {
		// The 1-th ms gets MPU-6050 data and calculate angle
		GetMpuData();
		AngleCalculate();
	} else if (g_nMainEventCount == 2) {
		// The 2-th ms is for angle control
		AngleControl();
	} else if (g_nMainEventCount == 3) {
		// The 3-th ms is for speed control
		
		// Outer control Mode
		g_nSpeedControlCount++;
		if (g_nSpeedControlCount >= 5) {
			// Do speed outer control for every 25ms
			SpeedOuterControl();
			g_nSpeedControlCount = 0;
			g_nSpeedControlPeriod = 0;
		}

		// Inner control Mode
		//SpeedInnerControl();
	} else if (g_nMainEventCount == 4) {
		// The 4-th ms output motor
		MotorOutput();
	}
}