/******************** (C) COPYRIGHT 2015 FTC ***************************
 * 作者		 ：FTC
 * 文件名  ：FTC_Motor.cpp
 * 描述    ：电机控制相关函数
**********************************************************************************/
#include "FTC_Motor.h"

FTC_Motor motor;

void FTC_Motor::writeMotor(uint16_t throttle, int32_t pidTermRoll, int32_t pidTermPitch, int32_t pidTermYaw)
{
	//to do
	float T_matrix[6][4];
	// #ifdef stupid
	// T_matrix[0]={0.1667,0.1667,0.2887,0.1667};
	// T_matrix[1]={0.1667,-0.1667,0.2887,0.1667};
	// T_matrix[2]={0.1667,-0.3333,0.0000,-0.1667};
	// T_matrix[3]={0.1667,-0.1667,-0.2887,0.1667};
	// T_matrix[4]={0.1667,0.1667,-0.2887,-0.1667};
	// T_matrix[5]={0.1667,0.3333,0.0000,0.1667};
	// #endif

	// // #ifdef not_stupid
	// T_matrix[0]={1,0.5,0.866,-1};
	// T_matrix[1]={1,-0.5,0.866,1};
	// T_matrix[2]={1,-1,0,-1};
	// T_matrix[3]={1,-0.5,-0.866,1};
	// T_matrix[4]={1,0.5,-0.866,-1};
	// T_matrix[5]={1,1,0,1};
	// // #endif
	for (u8 i=0;i<MAXMOTORS; i++)
		T_matrix[i][0]=1;
	for (u8 i=0;i<MAXMOTORS; i++) {
		T_matrix[i][1]=0.5;
		T_matrix[i][1]+=(i==2||i==5)?0.5:0;
		T_matrix[i][1]*=(i==1||i==3)?-1:1;
	}
	for (u8 i=0;i<MAXMOTORS; i++) {
		T_matrix[i][2]=(i==2||i==5)?0:-0.866;
		T_matrix[i][2]*=(i==3||i==4)?-1:1;
	}
	for (u8 i=0;i<MAXMOTORS; i++)
		T_matrix[i][3]=(i%2)?1:-1;

	for (u8 i=0;i<MAXMOTORS; i++)
		motorPWM[i]=T_matrix[i][0]*throttle+T_matrix[i][1]*pidTermRoll+T_matrix[i][2]*pidTermPitch+T_matrix[i][3]*pidTermYaw;

	int16_t maxMotor = motorPWM[0];
	for (u8 i = 1; i < MAXMOTORS; i++)
	{
		if (motorPWM[i] > maxMotor)
					maxMotor = motorPWM[i];				
	}
	
	for (u8 i = 0; i < MAXMOTORS; i++) 
	{
		if (maxMotor > MAXTHROTTLE)    
			motorPWM[i] -= maxMotor - MAXTHROTTLE;	
		//限制电机PWM的最小和最大值
		motorPWM[i] = constrain_uint16(motorPWM[i], MINTHROTTLE, MAXTHROTTLE);
	}

	//如果未解锁，则将电机输出设置为最低
	if(!ftc.f.ARMED)	
		ResetPWM();

	if(!ftc.f.ALTHOLD && rc.rawData[THROTTLE] < RC_MINCHECK)
		ResetPWM();

	//写入电机PWM
	pwm.SetPwm(motorPWM);
	
}

void FTC_Motor::getPWM(int16_t* pwm)
{
	*(pwm) = motorPWM[0];
	*(pwm+1) = motorPWM[1];
	*(pwm+2) = motorPWM[2];
	*(pwm+3) = motorPWM[3];
	*(pwm+4) = motorPWM[4];
	*(pwm+5) = motorPWM[5];	
}

void FTC_Motor::ResetPWM(void)
{
	for(u8 i=0; i< MAXMOTORS ; i++)
		motorPWM[i] = 1000;
}

/******************* (C) COPYRIGHT 2015 FTC *****END OF FILE************/
