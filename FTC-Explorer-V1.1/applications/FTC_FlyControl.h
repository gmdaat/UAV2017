#ifndef __FTC_FLYCONTROL_H
#define __FTC_FLYCONTROL_H

#include "FTC_Config.h"

#define FLYANGLE_MAX 200  //���������20��
#define MIN_THROWSTART_CHECK_1 5000 //���׸�Ӧ���ٶ�
#define MIN_THROWSTART_CHECK_2 12000 //���׸�Ӧ���ٶ�
#define MIN_THROWSTART_CHECK_3 18000 //���׸�Ӧ���ٶ�
#define MIN_THROWSTART_CHECK_4 24000 //���׸�Ӧ���ٶ�

enum {
    PIDROLL,
    PIDPITCH,
    PIDYAW,
    PIDANGLE,
    PIDMAG,
    PIDVELZ,
    PIDALT,
		PIDITEMS
};

 //���������������ȵ�����ֵ
static float threshold[4] = {MIN_THROWSTART_CHECK_1, MIN_THROWSTART_CHECK_2,
				MIN_THROWSTART_CHECK_3, MIN_THROWSTART_CHECK_4};

class FTC_FlyControl
{

public:
	
	FTC_PID pid[PIDITEMS];

	Vector3i setVelocity;
	uint8_t velocityControl;
	int32_t errorVelocityI;

	Vector3i velPIDTerm;

	int32_t AltHold;
	uint16_t ascendingTime;
	FTC_FlyControl();

	void PID_Reset(void);
	void AltHoldReset(void);

	//��̬�⻷����
	void Attitude_Outter_Loop(void);

	//��̬�ڻ�����
	void Attitude_Inner_Loop(void);

	//�߶��⻷����
	void Altitude_Outter_Loop(void);

	//�߶��ڻ�����
	void Altitude_Inner_Loop(void);

	//����״̬ʱ��ݼ�
	void ascendingTime_reduces(void);
private:
	
	uint8_t rollPitchRate;
	uint8_t yawRate;
	int32_t RateError[3];

	Vector3i velError;
	int16_t altHoldDeadband;

};

extern FTC_FlyControl fc;

#endif























