// control.h


#ifndef _CONTROL_H
#define _CONTROL_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "avr\pgmspace.h"

//�޷�
#define LIMIT 300
//��¼��ʷ����
#define LogNum 1
class Control
{


public:


	typedef struct
	{
		float VWp = 0;
		float VWi = 0;
		float VNp = 0;
		float VNi = 0;
		float VNd = 0;
		float Ve = 0;
		float Veg = 0;
		float KWp = 0;
		float KWi = 0;
		float KNp = 0;
		float KNi = 0;
		float KNd = 0;

	}LogV;

	typedef struct
	{
		//�Ƿ�ת
		bool IsFlip = 0;
		//��ǰ�Ƕ�
		float ypr = 0;
		//��ǰ���ٶ�
		float gyro = 0;
		//��ǰ���
		float out = 0;
		//�ϴ����
		float lastOut = 0;
		//�Ƕ�����ֵ
		float setpoint = 0;
		//�ڻ�PID
		float KpN = 0, KiN = 0, KdN = 0;
		//�⻷PI
		float KpW = 0, KiW = 0;
		//�ϴνǶ����
		float lastError = 0;
		//��ǰ�Ƕ����
		float error = 0;
		//�ϴν��ٶ����
		float lastErrorG = 0;
		//��ǰ���ٶ����
		float errorG = 0;
		//��ǰ�Ƕ�������
		float integ = 0;
		//��ǰ���ٶ�������
		float integG = 0;
		//pid������С
		float pidDeadband = 0;
		//�������
		unsigned long SampleTime = 0;
		//������ʱ��
		unsigned long lastTime = 0;
		//��ǰ��¼��
		int logNum = 0;
		//��ʷ�����¼
		PROGMEM LogV logV[LogNum];
	}ConfigPID;


	void Compute(ConfigPID *pid);
	void SetPID(ConfigPID *pid);
private:
};
#endif

