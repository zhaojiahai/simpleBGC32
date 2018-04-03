///=====================================================================================================
// AHRS.c
// S.O.H. Madgwick
// 25th August 2010
//
//  1 June 2012 Modified by J. Ihlein
// 27 Aug  2012 Extensively modified to include G.K. Egan's accel confidence calculations and
//                                                          calculation efficiency updates
//=====================================================================================================
// Description:
//
// Quaternion implementation of the 'DCM filter' [Mayhony et al].  Incorporates the magnetic distortion
// compensation algorithms from my filter [Madgwick] which eliminates the need for a reference
// direction of flux (bx bz) to be predefined and limits the effect of magnetic distortions to yaw
// axis only.
//
// User must define 'halfT' as the (sample period / 2), and the filter gains 'Kp' and 'Ki'.
//
// Global variables 'q0', 'q1', 'q2', 'q3' are the quaternion elements representing the estimated
// orientation.  See my report for an overview of the use of quaternions in this application.
//
// User must call 'AHRSupdate()' every sample period and parse calibrated gyroscope ('gx', 'gy', 'gz'),
// accelerometer ('ax', 'ay', 'ay') and magnetometer ('mx', 'my', 'mz') data.  Gyroscope units are
// radians/second, accelerometer and magnetometer units are irrelevant as the vector is normalised.
//
//=====================================================================================================

//----------------------------------------------------------------------------------------------------
// Header files

#include "board.h"

//----------------------------------------------------------------------------------------------------
// Variable definitions

float exAcc    = 0.0f,    eyAcc = 0.0f,    ezAcc = 0.0f; // accel error
float exAccInt = 0.0f, eyAccInt = 0.0f, ezAccInt = 0.0f; // accel integral error

float exMag    = 0.0f, eyMag    = 0.0f, ezMag    = 0.0f; // mag error
float exMagInt = 0.0f, eyMagInt = 0.0f, ezMagInt = 0.0f; // mag integral error

float kpAcc, kiAcc;

float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

// auxiliary variables to reduce number of repeated operations
float q0q0, q0q1, q0q2, q0q3;
float q1q1, q1q2, q1q3;
float q2q2, q2q3;
float q3q3;

float halfT;

uint8_t MargAHRSinitialized = false;

//----------------------------------------------------------------------------------------------------

float accConfidenceDecay = 0.0f;
float accConfidence      = 1.0f;

#define HardFilter(O,N)  ((O)*0.9f+(N)*0.1f)

void calculateAccConfidence(float accMag)
{
	// G.K. Egan (C) computes confidence in accelerometers when
	// aircraft is being accelerated over and above that due to gravity

	static float accMagP = 1.0f;

	accMag /= accelOneG;  // HJI Added to convert MPS^2 to G's

	accMag  = HardFilter(accMagP, accMag);
	accMagP = accMag;

	accConfidence = constrain(1.0f - (accConfidenceDecay * sqrt(fabs(accMag - 1.0f))), 0.0f, 1.0f);
}

//----------------------------------------------------------------------------------------------------

//====================================================================================================
// Initialization
//====================================================================================================
//航姿参考系统初始化
void MargAHRSinit(float ax, float ay, float az, float mx, float my, float mz)
{
	float initialRoll, initialPitch;
	float cosRoll, sinRoll, cosPitch, sinPitch;
	float magX, magY;
	float initialHdg, cosHeading, sinHeading;


	//使用加速度数据计算欧拉角 ，滚转角和俯仰角
	initialRoll  = atan2(-ay, -az);
	initialPitch = atan2(ax, -az);


	//对欧拉角进行余弦和正弦计算，分别把计算结果保存下来
	cosRoll  = cosf(initialRoll);
	sinRoll  = sinf(initialRoll);
	cosPitch = cosf(initialPitch);
	sinPitch = sinf(initialPitch);

	magX = 1.0f;  // HJI mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;

	magY = 0.0f;  // HJI my * cosRoll - mz * sinRoll;

	initialHdg = atan2f(-magY, magX);//解算航向角

	cosRoll = cosf(initialRoll * 0.5f);
	sinRoll = sinf(initialRoll * 0.5f);

	cosPitch = cosf(initialPitch * 0.5f);
	sinPitch = sinf(initialPitch * 0.5f);

	cosHeading = cosf(initialHdg * 0.5f);
	sinHeading = sinf(initialHdg * 0.5f);


	//得到四元数
	q0 = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
	q1 = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
	q2 = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
	q3 = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;


	//把计算参考方向用到的值先都计算好,减少重复计算,因为MargAHRSupdate函数里面要用到。
	// auxillary variables to reduce number of repeated operations, for 1st pass
	q0q0 = q0 * q0;
	q0q1 = q0 * q1;
	q0q2 = q0 * q2;
	q0q3 = q0 * q3;
	q1q1 = q1 * q1;
	q1q2 = q1 * q2;
	q1q3 = q1 * q3;
	q2q2 = q2 * q2;
	q2q3 = q2 * q3;
	q3q3 = q3 * q3;
}

//====================================================================================================
// Function
//====================================================================================================
//航姿参考系统更新
void MargAHRSupdate(float gx, float gy, float gz,
                    float ax, float ay, float az,
                    float mx, float my, float mz,
                    uint8_t magDataUpdate, float dt)
{
	float norm, normR;
	float hx, hy, hz, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float q0i, q1i, q2i, q3i;

	//-------------------------------------------

	if ((MargAHRSinitialized == false)) // HJI && (magDataUpdate == true))
	{

		//如果航姿参考系统参数还没有初始化过，那么执行AHRS初始化
		MargAHRSinit(ax, ay, az, mx, my, mz);

		MargAHRSinitialized = true;//标记航姿参考系统参数已经初始化过
	}

	//-------------------------------------------

	if (MargAHRSinitialized == true)//如果航姿参考系统参数已经初始化过
	{
		halfT = dt * 0.5f;//半周期，求解四元数微分方程时用得到。

		norm = sqrt(SQR(ax) + SQR(ay) + SQR(az));//加速度归一化

		if (norm != 0.0f)//如果归一化后的模等于0 ，那么说明加速度数据或者传感器不正常，正常情况下 归一化后的结果恒等于 1.0 ，这是重点。
		{
			calculateAccConfidence(norm);//由于处于运动状态，所有要计算加速度数据归一化后的可信度
			kpAcc = eepromConfig.KpAcc * accConfidence; //加速度比例系数 * 可信度
			kiAcc = eepromConfig.KiAcc * accConfidence;//加速度积分系数 * 可信度

			normR = 1.0f / norm; //加速度归一化
			ax *= normR;
			ay *= normR;
			az *= normR;

			// estimated direction of gravity (v)
			vx = 2.0f * (q1q3 - q0q2);//计算方向余弦矩阵
			vy = 2.0f * (q0q1 + q2q3);
			vz = q0q0 - q1q1 - q2q2 + q3q3;

			// error is sum of cross product between reference direction
			// of fields and direction measured by sensors

			//误差是由传感器测量的参考方向与方向之间的叉积,由此

			//得到一个误差向量，通过这个误差向量来修正陀螺仪数据。
			exAcc = vy * az - vz * ay;
			eyAcc = vz * ax - vx * az;
			ezAcc = vx * ay - vy * ax;



			gx += exAcc * kpAcc;//比例增益控制加速度计的收敛速度
			gy += eyAcc * kpAcc;
			gz += ezAcc * kpAcc;

			if (kiAcc > 0.0f)//用积分增益控制陀螺仪的偏差收敛速率
			{
				exAccInt += exAcc * kiAcc;
				eyAccInt += eyAcc * kiAcc;
				ezAccInt += ezAcc * kiAcc;

				gx += exAccInt;
				gy += eyAccInt;
				gz += ezAccInt;
			}
		}

		//-------------------------------------------

		norm = sqrt(SQR(mx) + SQR(my) + SQR(mz));//三轴磁力计归一化

		if ((magDataUpdate == true) && (norm != 0.0f))//如果入口参数magDataUpdate == true并且归一化的结果norm不是0，才对磁力计数据进行更新计算
		{
			normR = 1.0f / norm;//三轴磁场归一化
			mx *= normR;
			my *= normR;
			mz *= normR;

			// compute reference direction of flux

			//计算参考方向
			hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));

			hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));

			hz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

			bx = sqrt((hx * hx) + (hy * hy));

			bz = hz;

			// estimated direction of flux (w)

			//根据参考方向估计云台机体方向
			wx = 2.0f * (bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2));

			wy = 2.0f * (bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3));

			wz = 2.0f * (bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2));

			exMag = my * wz - mz * wy;//三轴磁场和估计方向进行叉积运算,计算估计方向与三轴磁场的偏差
			eyMag = mz * wx - mx * wz;
			ezMag = mx * wy - my * wx;

			// use un-extrapolated old values between magnetometer updates
			// dubious as dT does not apply to the magnetometer calculation so
			// time scaling is embedded in KpMag and KiMag

			//使用估计的旧值与磁力计值进行更新，dT不能应用在磁力计计算中，因此时间被嵌入在KpMag 和 KiMag里面
			gx += exMag * eepromConfig.KpMag;//比例增益控制磁强计收敛速度
			gy += eyMag * eepromConfig.KpMag;
			gz += ezMag * eepromConfig.KpMag;



			if (eepromConfig.KiMag > 0.0f)//用积分增益控制陀螺仪的偏差收敛速率
			{
				exMagInt += exMag * eepromConfig.KiMag;
				eyMagInt += eyMag * eepromConfig.KiMag;
				ezMagInt += ezMag * eepromConfig.KiMag;

				gx += exMagInt;
				gy += eyMagInt;
				gz += ezMagInt;
			}
		}

		//-------------------------------------------

		// integrate quaternion rate

		//四元数微分方程，其中halfT为测量周期，g为陀螺仪角速度，其余都是已知量，这里使用了一阶龙格库塔法求解四元数微分方程。
		q0i = (-q1 * gx - q2 * gy - q3 * gz) * halfT;
		q1i = (q0 * gx + q2 * gz - q3 * gy) * halfT;
		q2i = (q0 * gy - q1 * gz + q3 * gx) * halfT;
		q3i = (q0 * gz + q1 * gy - q2 * gx) * halfT;
		q0 += q0i;
		q1 += q1i;
		q2 += q2i;
		q3 += q3i;

		// normalise quaternion

		//四元数归一化，为什么又要归一化呢？这是因为引入了误差向量后四元数失去了规范性了(模不等于1了),所以要重新归一化
		normR = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
		q0 *= normR;
		q1 *= normR;
		q2 *= normR;
		q3 *= normR;

		// auxiliary variables to reduce number of repeated operations

		//把计算参考方向用到的值先都计算好,减少下面计算欧拉角时候的重复计算。
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;


		//最后根据四元数方向余弦阵和欧拉角的转换关系，把四元数转换成欧拉角
		sensors.margAttitude500Hz[ROLL ] = atan2f(2.0f * (q0q1 + q2q3), q0q0 - q1q1 - q2q2 + q3q3);
		sensors.margAttitude500Hz[PITCH] = -asinf(2.0f * (q1q3 - q0q2));
		sensors.margAttitude500Hz[YAW  ] = atan2f(2.0f * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3);
	}
}

//====================================================================================================
// END OF CODE
//====================================================================================================
