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


	//当前角度误差 = 期望角度 - 当前角度
	//外环PID_P = 外环KP * 当前角度误差
	//当前角度误差积分及其积分限幅
	//外环PID_I  = 外环KI * 当前角度误差积分
	//外环PID_输出 = 外环PID_P + 外环PID_I
	//
	//当前角速度误差 = 外环PID_输出 - 当前角速度(直接用陀螺仪输出)
	//内环PID_P = 内环KP * 当前角速度误差
	//当前角速度误差积分及其积分限幅
	//内环PID_I = 内环KI * 当前角速度误差积分
	//当前角速度的微分(当前角速度误差 - 上次角速度误差)
	//内环PID_D = 内环KD * 当前角速度的微分
	//内环PID_输出 = 内环PID_P + 内环PID_I + 内环 PID_D


	unsigned long now = millis();
	unsigned long timeChange = (now - configPID->lastTime);
	if (timeChange >= configPID->SampleTime)
	{


		configPID->lastOut = configPID->out;
		configPID->error = configPID->setpoint - configPID->ypr;
		//pid死区
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
	//全部记录大于死区或全部小于死区
	if (i1 == LIMIT || i2 == LIMIT)
	{
		//KpW太小需要加大
		configPID->KpW -= 0.01;

	}
	else
	{
		//KpW太大需要减小
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
	////全部记录大于死区或全部小于死区
	//if (i1 == LIMIT || i2 == LIMIT)
	//{
	//	//KpW太小需要加大
	//	configPID->KpN += 0.001;

	//}
	//else
	//{
	//	//KpW太大需要减小
	//	configPID->KpN -= 0.001;

	//}
}
