// control.h


#ifndef _CONTROL_H
#define _CONTROL_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "avr\pgmspace.h"

//限幅
#define LIMIT 300
//记录历史数量
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
		//是否翻转
		bool IsFlip = 0;
		//当前角度
		float ypr = 0;
		//当前角速度
		float gyro = 0;
		//当前输出
		float out = 0;
		//上次输出
		float lastOut = 0;
		//角度期望值
		float setpoint = 0;
		//内环PID
		float KpN = 0, KiN = 0, KdN = 0;
		//外环PI
		float KpW = 0, KiW = 0;
		//上次角度误差
		float lastError = 0;
		//当前角度误差
		float error = 0;
		//上次角速度误差
		float lastErrorG = 0;
		//当前角速度误差
		float errorG = 0;
		//当前角度误差积分
		float integ = 0;
		//当前角速度误差积分
		float integG = 0;
		//pid死区大小
		float pidDeadband = 0;
		//采样间隔
		unsigned long SampleTime = 0;
		//最后更新时间
		unsigned long lastTime = 0;
		//当前记录数
		int logNum = 0;
		//历史结果记录
		PROGMEM LogV logV[LogNum];
	}ConfigPID;


	void Compute(ConfigPID *pid);
	void SetPID(ConfigPID *pid);
private:
};
#endif

