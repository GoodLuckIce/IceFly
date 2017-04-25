
//������libraries ����������ļ�����ͷ����Ч��

#include "Wire.h"
#include "avr\pgmspace.h"
#include "math.h"
#include <SoftwareSerial.h>

#include "define.h"


//��ѹ��
#include "SFE_BMP180.h"

//��������
#include "HMC5883L.h"

//�˶�������
#include "freeram.h"
#include "mpu.h"
#include "I2Cdev.h"

#include "control.h"

//�Ƿ����
#define isFlyPin 12

//�Ƿ����õ��
#define isSetDianTiaoPin 13

//����
#define PWMLPin 11
//�ҵ��
#define PWMRPin 9
//ǰ���
#define PWMFPin 10
//����
#define PWMHPin 3


//��������
//HMC5883L compass;

//���ת��
int speedL = 0;//����
int speedR = 0;//�ҵ��
int speedF = 0;//ǰ���
int speedH = 0;//����

			   //PWMת���޷�
#define SPEED_MIN 100
#define SPEED_MXA 254

			   //��ѹ��
			   //SFE_BMP180 pressure;
			   //double baseline; // baseline pressure

Control *control;

Control::ConfigPID configPIDLR;
Control::ConfigPID configPIDFH;

SoftwareSerial mySerial(0, 1); // RX, TX

String inputString = "";
boolean stringComplete = false;
void setup()
{
	//����I2cͨ��Ƶ��
	Fastwire::setup(500, 0);

	//��ʼ��MPU,����DMP����
	mympu_open(200);

	pinMode(isFlyPin, INPUT);
	pinMode(isSetDianTiaoPin, INPUT);

	pinMode(PWMLPin, OUTPUT);
	pinMode(PWMRPin, OUTPUT);
	pinMode(PWMFPin, OUTPUT);
	pinMode(PWMHPin, OUTPUT);

	analogWrite(PWMLPin, 100);
	analogWrite(PWMRPin, 100);
	analogWrite(PWMFPin, 100);
	analogWrite(PWMHPin, 100);


	//��ѹ��
	/*while (!pressure.begin())
	{
	mySerial.println("BMP180 init fail (disconnected?)\n\n");
	delay(500);
	}*/

	configPIDLR.KpW = 0;
	configPIDLR.KiW = 0;
	configPIDLR.KpN = 0;
	configPIDLR.KiN = 0;
	configPIDLR.KdN = 0;

	configPIDFH.KpW = 1.5;							  //1.5;
	configPIDFH.KiW = 0.01;							  //0.01;
	configPIDFH.KpN = 0.4;							  //0.4;
	configPIDFH.KiN = 0.8;							  //0.01;
	configPIDFH.KdN = 1;							  //0.8;
	configPIDFH.IsFlip = 1;							  // = 1;


	mySerial.begin(115200);
	Serial.begin(115200);
	pinMode(12, OUTPUT);
	pinMode(13, OUTPUT);
}

int baseServo = 100;
int isFly = 0;
int isSetDianTiao = 0;
char strOrderTemp[15];
char strOrder[15];
char strOrderOld[15];
String serialStringOld = "2100";
String serialStringTemp[10];
int serialStringi = 0;
void loop()
{
	while (mySerial.available())
	{
		char inChar = (char)mySerial.read();
		if (inChar == '\n') {
			stringComplete = true;
			break;
		}
		inputString += inChar;
	}

	if (stringComplete)
	{

		if (inputString.length() == 5 || inputString.length() == 11)
		{
			serialStringTemp[serialStringi] = inputString;
			serialStringi = serialStringi == 9 ? 0 : ++serialStringi;

			if (serialStringTemp[0] == serialStringTemp[1] &&
				serialStringTemp[0] == serialStringTemp[2] &&
				serialStringTemp[0] == serialStringTemp[3] &&
				serialStringTemp[0] == serialStringTemp[4] &&
				serialStringTemp[0] == serialStringTemp[5] &&
				serialStringTemp[0] == serialStringTemp[6] &&
				serialStringTemp[0] == serialStringTemp[7] &&
				serialStringTemp[0] == serialStringTemp[8] &&
				serialStringTemp[0] == serialStringTemp[9])
			{
				serialStringOld = inputString;
			}
			//strncpy(strOrderOld, strOrder, sizeof(strOrder));
		}

		/*digitalWrite(12, LOW);
		digitalWrite(13, HIGH);
		delay(50);
		digitalWrite(13, LOW);
		delay(50);*/
		serialStringOld.toCharArray(strOrderTemp, sizeof(strOrderTemp));
		long orderLong = atol(strOrderTemp);
		if (orderLong > 2000)
		{
			ltoa(orderLong, strOrder, 10);
		}

		inputString = "";
		stringComplete = false;
	}

	//Serial.println(millis());

	if (strOrder[0] == *"1")//��һλ"1"������� �͵��������Ƕ� ��1000000100
	{
		char LRtemp[3];
		LRtemp[0] = strOrder[2];
		LRtemp[1] = strOrder[3];
		int LRsetpoint = atoi(LRtemp);
		//����ڶ�λ����1 �����ҷ�
		if (strOrder[1] == *"1")
		{
			configPIDLR.setpoint = LRsetpoint;
		}
		else if (strOrder[1] == *"2")
		{
			configPIDLR.setpoint = -LRsetpoint;
		}
		else
		{
			configPIDLR.setpoint = 0;
		}

		char FHtemp[3];
		FHtemp[0] = strOrder[5];
		FHtemp[1] = strOrder[6];
		int FHsetpoint = atoi(FHtemp);
		//�������λ����1 ��ǰ���
		if (strOrder[4] == *"1")
		{
			configPIDFH.setpoint = FHsetpoint;
		}
		else if (strOrder[4] == *"2")
		{
			configPIDFH.setpoint = -FHsetpoint;
		}
		else
		{
			configPIDFH.setpoint = 0;
		}


		//��ȡ��������
		char baseServoTemp[3];
		baseServoTemp[0] = strOrder[7];
		baseServoTemp[1] = strOrder[8];
		baseServoTemp[2] = strOrder[9];
		baseServo = atoi(baseServoTemp);
		baseServo = baseServo > 255 ? 100 : baseServo;
		baseServo = baseServo < 100 ? 100 : baseServo;
		if (baseServo > 100)
		{
			isFly = 1;
		}
		else
		{
			isFly = 0;
		}
	}
	else if (strOrder[0] == *"2")//��һλ"2"������� ���������� ��2100
	{
		//��ȡ��������
		char baseServoTemp[3];
		baseServoTemp[0] = strOrder[1];
		baseServoTemp[1] = strOrder[2];
		baseServoTemp[2] = strOrder[3];
		baseServo = atoi(baseServoTemp);
		baseServo = baseServo > 255 ? 100 : baseServo;
		baseServo = baseServo < 100 ? 100 : baseServo;
		if (baseServo > 100)
		{
			isFly = 1;
			isSetDianTiao = 0;
		}
		else
		{
			isFly = 0;
		}


	}
	else if (strOrder[0] == *"3")//��һλ"3"�������õ�� ��2100
	{
		//��ȡ��������
		char baseServoTemp[3];
		baseServoTemp[0] = strOrder[1];
		baseServoTemp[1] = strOrder[2];
		baseServoTemp[2] = strOrder[3];
		baseServo = atoi(baseServoTemp);
		isSetDianTiao = 1;
		isFly = 0;
	}

	//���
	Fly();


	//// ��ѹ��
	//// Get the baseline pressure:

	//baseline = getPressure();

	//mySerial.print("baseline pressure: ");
	//mySerial.print(baseline);
	//mySerial.println(" mb");
	//double a, P;

	//// Get a new pressure reading:

	//P = getPressure();

	//// Show the relative altitude difference between
	//// the new reading and the baseline reading:

	//a = pressure.altitude(P, baseline);

	//mySerial.print("relative altitude: ");
	//if (a >= 0.0) mySerial.print(" "); // add a space for positive numbers
	//mySerial.print(a, 1);
	//mySerial.print(" meters, ");
	//if (a >= 0.0) mySerial.print(" "); // add a space for positive numbers
	//mySerial.print(a*3.28084, 0);
	//mySerial.println(" feet");

}


void Fly()
{
	if (isFly)
	{
		//��ȡ��̬
		mympu_update();

		//תͷ��б
		float LFRH = mympu.ypr[0];
		//������бconfigPIDRoll
		float LR = mympu.ypr[1];
		configPIDLR.ypr = LR;
		//ǰ����бconfigPIDPitch
		float FH = mympu.ypr[2];
		configPIDFH.ypr = FH;

		float GLFR = mympu.gyro[0];

		float GLR = mympu.gyro[1];
		configPIDLR.gyro = GLR;

		float GFH = mympu.gyro[2];
		configPIDFH.gyro = GFH;

		/*mySerial.print(LFRH);
		mySerial.print("  ");
		mySerial.print(LR);
		mySerial.print("  ");
		mySerial.print(FH);
		mySerial.print("  ");
		mySerial.print(GLFR);
		mySerial.print("  ");
		mySerial.print(GLR);
		mySerial.print("  ");
		mySerial.print(GFH);
		mySerial.print("  ");
		mySerial.println();*/

		control->Compute(&configPIDLR);
		control->Compute(&configPIDFH);


		int baseServoTemp = map(baseServo, SPEED_MIN, SPEED_MXA, -LIMIT, LIMIT);

		int speedL = map(baseServoTemp - configPIDLR.out, -LIMIT, LIMIT, SPEED_MIN, SPEED_MXA);
		int speedR = map(baseServoTemp + configPIDLR.out, -LIMIT, LIMIT, SPEED_MIN, SPEED_MXA);
		int speedF = map(baseServoTemp - configPIDFH.out, -LIMIT, LIMIT, SPEED_MIN, SPEED_MXA);
		int speedH = map(baseServoTemp + configPIDFH.out, -LIMIT, LIMIT, SPEED_MIN, SPEED_MXA);

		speedL = abs(ceil(speedL));
		speedR = abs(ceil(speedR));
		speedF = abs(ceil(speedF));
		speedH = abs(ceil(speedH));

		speedL = speedL > 254 ? 254 : speedL;
		speedR = speedR > 254 ? 254 : speedR;
		speedF = speedF > 254 ? 254 : speedF;
		speedH = speedH > 254 ? 254 : speedH;

		speedL = speedL < 100 ? 100 : speedL;
		speedR = speedR < 100 ? 100 : speedR;
		speedF = speedF < 100 ? 100 : speedF;
		speedH = speedH < 100 ? 100 : speedH;

		analogWrite(PWMLPin, 100);
		analogWrite(PWMRPin, 100);
		analogWrite(PWMFPin, speedF);
		analogWrite(PWMHPin, speedH);


		/*mySerial.print(configPIDLR.ypr);
		mySerial.print("  ");
		mySerial.print(configPIDLR.out);
		mySerial.print("  ");
		mySerial.print(speedL);
		mySerial.print("  ");
		mySerial.print(speedR);
		mySerial.println();*/

	}
	else if (isSetDianTiao)
	{
		analogWrite(PWMLPin, baseServo);
		analogWrite(PWMRPin, baseServo);
		analogWrite(PWMFPin, baseServo);
		analogWrite(PWMHPin, baseServo);
	}
	else
	{
		analogWrite(PWMLPin, 100);
		analogWrite(PWMRPin, 100);
		analogWrite(PWMFPin, 100);
		analogWrite(PWMHPin, 100);
	}
}


//
//double getPressure()
//{
//	char status;
//	double T, P, p0, a;
//
//	// You must first get a temperature measurement to perform a pressure reading.
//
//	// Start a temperature measurement:
//	// If request is successful, the number of ms to wait is returned.
//	// If request is unsuccessful, 0 is returned.
//
//	status = pressure.startTemperature();
//	if (status != 0)
//	{
//		// Wait for the measurement to complete:
//
//		delay(status);
//
//		// Retrieve the completed temperature measurement:
//		// Note that the measurement is stored in the variable T.
//		// Use '&T' to provide the address of T to the function.
//		// Function returns 1 if successful, 0 if failure.
//
//		status = pressure.getTemperature(T);
//		if (status != 0)
//		{
//			// Start a pressure measurement:
//			// The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
//			// If request is successful, the number of ms to wait is returned.
//			// If request is unsuccessful, 0 is returned.
//
//			status = pressure.startPressure(3);
//			if (status != 0)
//			{
//				// Wait for the measurement to complete:
//				delay(status);
//
//				// Retrieve the completed pressure measurement:
//				// Note that the measurement is stored in the variable P.
//				// Use '&P' to provide the address of P.
//				// Note also that the function requires the previous temperature measurement (T).
//				// (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
//				// Function returns 1 if successful, 0 if failure.
//
//				status = pressure.getPressure(P, T);
//				if (status != 0)
//				{
//					return(P);
//				}
//				else mySerial.println("error retrieving pressure measurement\n");
//			}
//			else mySerial.println("error starting pressure measurement\n");
//		}
//		else mySerial.println("error retrieving temperature measurement\n");
//	}
//	else
//	{
//		mySerial.println("error starting temperature measurement\n");
//	}
//	return 0;
//}
//
//

