/******************** (C) COPYRIGHT 2015 FTC ***************************
 * 作者		 ：FTC
 * 文件名  ：FTC_FlyControl.cpp
 * 描述    ：飞行控制
**********************************************************************************/
#include "FTC_FlyControl.h"

FTC_FlyControl fc; 

float getDeltaT(uint32_t currentT) {
	static uint32_t previousT; 
	float deltaT = (currentT - previousT) * 1e-6; 
	previousT = currentT; 

	return deltaT; 
}

FTC_FlyControl::FTC_FlyControl() {
	rollPitchRate = 150; 
	yawRate = 50; 

	altHoldDeadband = 100; 

	//重置PID参数
	PID_Reset(); 
}

//重置PID参数
void FTC_FlyControl::PID_Reset(void) {
	pid[PIDROLL].set_pid(0.15, 0.15, 0.02, 200); 
	pid[PIDPITCH].set_pid(0.15, 0.15, 0.02, 200); 
	pid[PIDYAW].set_pid(0.8, 0.45, 0, 200); 
	pid[PIDANGLE].set_pid(5, 0, 0, 0); 
	pid[PIDMAG].set_pid(2, 0, 0, 0); 
	pid[PIDVELZ].set_pid(1.5, 0.5, 0.002, 150); 
	pid[PIDALT].set_pid(1.2, 0, 0, 200); 
}

//飞行器姿态外环控制
void FTC_FlyControl::Attitude_Outter_Loop(void) {
	out_ans[PIDROLL] = pid[PIDROLL].get_p(rc.Command[PIDROLL] - imu.angle.x); 
	out_ans[PIDPITCH] = pid[PIDPITCH].get_p(rc.Command[PIDPITCH] - imu.angle.y); 
	out_ans[PIDYAW] = pid[PIDYAW].get_p(rc.Command[PIDYAW] - imu.angle.z); 
}

//飞行器姿态内环控制
void FTC_FlyControl::Attitude_Inner_Loop(void) {
	static float deltaT = PID_INNER_LOOP_TIME * 1e-6; 
	inner_ans[PIDROLL] = pid[PIDROLL].get_pid(out_ans[PIDROLL] - imu.Gyro_lpf.x, deltaT); 
	inner_ans[PIDPITCH] = pid[PIDPITCH].get_pid(out_ans[PIDPITCH] - imu.Gyro_lpf.y, deltaT); 
	inner_ans[PIDROLL] = pid[PIDROLL].get_pid(out_ans[PIDYAW] - imu.Gyro_lpf.z, deltaT); 

	if (rc.Command[THROTTLE] < RC_MINTHROTTLE) {
		inner_ans[PIDROLL].reset_I(); 
		inner_ans[PIDPITCH].reset_I(); 
		inner_ans[PIDROLL].reset_I(); 
	}

	motor.writeMotor(getThrottleCom(rc.Command[THROTTLE]), inner_ans[PIDROLL], inner_ans[PIDPITCH], inner_ans[PIDYAW]); 
}

//飞行器高度外环控制
void FTC_FlyControl::Altitude_Outter_Loop(void) {
}

//飞行器高度内环控制
void FTC_FlyControl::Altitude_Inner_Loop(void) {
}

void FTC_FlyControl::AltHoldReset(void) {
	AltHold = nav.position.z; 
}

uint16_t FTC_FlyControl::getThrottleCom(int16_t throttle) {
	return (uint16_t)((float)throttle / (imu.angle.x > imu.angle.y?imu.angle.x:imu.angle.y)); 
}

/************************ (C) COPYRIGHT 2015 FTC *****END OF FILE**********************/