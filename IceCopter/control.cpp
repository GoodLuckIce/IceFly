// 
// 
// 

#include "control.h"

void Control::Compute(ConfigPID *configPID)
{
	if (configPID->IsFlip)
	{
		configPID->ypr = configPID->ypr > 0 ? configPID->ypr - 180 : configPID->ypr + 180;
	}


	//��ǰ�Ƕ���� = �����Ƕ� - ��ǰ�Ƕ�
	//�⻷PID_P = �⻷KP * ��ǰ�Ƕ����
	//��ǰ�Ƕ������ּ�������޷�
	//�⻷PID_I  = �⻷KI * ��ǰ�Ƕ�������
	//�⻷PID_��� = �⻷PID_P + �⻷PID_I
	//
	//��ǰ���ٶ���� = �⻷PID_��� - ��ǰ���ٶ�(ֱ�������������)
	//�ڻ�PID_P = �ڻ�KP * ��ǰ���ٶ����
	//��ǰ���ٶ������ּ�������޷�
	//�ڻ�PID_I = �ڻ�KI * ��ǰ���ٶ�������
	//��ǰ���ٶȵ�΢��(��ǰ���ٶ���� - �ϴν��ٶ����)
	//�ڻ�PID_D = �ڻ�KD * ��ǰ���ٶȵ�΢��
	//�ڻ�PID_��� = �ڻ�PID_P + �ڻ�PID_I + �ڻ� PID_D


	unsigned long now = millis();
	unsigned long timeChange = (now - configPID->lastTime);
	if (timeChange >= configPID->SampleTime)
	{


		configPID->lastOut = configPID->out;
		configPID->error = configPID->setpoint - configPID->ypr;
		//pid����
		if (fabs(configPID->error) > configPID->pidDeadband)
		{

			if (configPID->logNum == LogNum)
			{
				//SetPID(configPID);
				configPID->logNum = 0;
			}

			float Wpid_p = configPID->KpW * configPID->error;
			configPID->integ += configPID->error;
			configPID->integ = configPID->integ > LIMIT ? LIMIT : configPID->integ;
			configPID->integ = configPID->integ < -LIMIT ? -LIMIT : configPID->integ;
			float Wpid_i = configPID->KiW * configPID->integ;
			float Wpid_Out = Wpid_p + Wpid_i;


			configPID->errorG = configPID->gyro + Wpid_Out;
			float Npid_p = configPID->KpN * configPID->errorG;
			configPID->integG += configPID->errorG;
			configPID->integG = configPID->integG > LIMIT ? LIMIT : configPID->integG;
			configPID->integG = configPID->integG < -LIMIT ? -LIMIT : configPID->integG;
			float Npid_i = configPID->KiN * configPID->integG;
			float Npid_d = configPID->KdN * (configPID->errorG - configPID->lastErrorG);
			configPID->out = Npid_p + Npid_i + Npid_d;

			configPID->lastError = configPID->error;
			configPID->lastErrorG = configPID->errorG;

			configPID->out = configPID->out > LIMIT ? LIMIT : configPID->out;
			configPID->out = configPID->out < -LIMIT ? -LIMIT : configPID->out;

			configPID->logV[configPID->logNum].VWp = Wpid_p;
			configPID->logV[configPID->logNum].VWi = Wpid_i;
			configPID->logV[configPID->logNum].VNp = Npid_p;
			configPID->logV[configPID->logNum].VNi = Npid_i;
			configPID->logV[configPID->logNum].VNd = Npid_d;
			configPID->logV[configPID->logNum].KWp = configPID->KpW;
			configPID->logV[configPID->logNum].KWi = configPID->KiW;
			configPID->logV[configPID->logNum].KNp = configPID->KpN;
			configPID->logV[configPID->logNum].KNi = configPID->KiN;
			configPID->logV[configPID->logNum].KNd = configPID->KdN;
			configPID->logV[configPID->logNum].Ve = configPID->error;
			configPID->logV[configPID->logNum].Veg = configPID->errorG;
			configPID->logNum++;
		}
		configPID->lastTime = now;

	}
}


void Control::SetPID(ConfigPID *configPID)
{
	int i1 = 0;
	for (i1 = 0; i1 < LIMIT; i1++)
	{
		if (!(configPID->logV[i1].Ve > 0))
		{
			break;
		}
	}
	int i2 = 0;
	for (i2 = 0; i2 < LIMIT; i2++)
	{
		if (!(configPID->logV[i2].Ve < 0))
		{
			break;
		}
	}
	//ȫ����¼����������ȫ��С������
	if (i1 == LIMIT || i2 == LIMIT)
	{
		//KpW̫С��Ҫ�Ӵ�
		configPID->KpW -= 0.01;

	}
	else
	{
		//KpW̫����Ҫ��С
		configPID->KpW += 0.01;

	}


	//i1 = 0;
	//for (i1 = 0; i1 < LIMIT; i1++)
	//{
	//	if (!(configPID->logV[i1].Veg > 0))
	//	{
	//		break;
	//	}
	//}
	//i2 = 0;
	//for (i2 = 0; i2 < LIMIT; i2++)
	//{
	//	if (!(configPID->logV[i2].Veg < 0))
	//	{
	//		break;
	//	}
	//}
	////ȫ����¼����������ȫ��С������
	//if (i1 == LIMIT || i2 == LIMIT)
	//{
	//	//KpW̫С��Ҫ�Ӵ�
	//	configPID->KpN += 0.001;

	//}
	//else
	//{
	//	//KpW̫����Ҫ��С
	//	configPID->KpN -= 0.001;

	//}
}
