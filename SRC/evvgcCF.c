///////////////////////////////////////////////////////////////////////////////

#include "board.h"

///////////////////////////////////////////////////////////////////////////////

float accAngleSmooth[3];

///////////////////////////////////////////////////////////////////////////////

void initOrientation()
{
	int initLoops = 150;
	float accAngle[NUMAXIS] = { 0.0f, 0.0f, 0.0f };
	int i;

	for (i = 0; i < initLoops; i++)
	{
		readMPU6050();//从MPU6050得到加速度和陀螺仪数据，并进行 与初始化方位估计矩阵（根据IMU单元的方位确定的矩阵A ） 相乘后的数据

		computeMPU6050TCBias();//计算温度补偿偏差值

		//（矩阵相乘后的加速度数据-温度补偿偏差）* （(1/8192) * 9.8065）
		//(1/8192) * 9.8065  (8192 LSB = 1 G)
		//1G量程的8192个数字量分之1，对应重力加速度9.8065m/1G的8192分之1
		sensors.accel500Hz[XAXIS] = ((float)rawAccel[XAXIS].value - accelTCBias[XAXIS]) * ACCEL_SCALE_FACTOR;
		sensors.accel500Hz[YAXIS] = ((float)rawAccel[YAXIS].value - accelTCBias[YAXIS]) * ACCEL_SCALE_FACTOR;
		sensors.accel500Hz[ZAXIS] = -((float)rawAccel[ZAXIS].value - accelTCBias[ZAXIS]) * ACCEL_SCALE_FACTOR;

		//进行欧拉角积分运算
		accAngle[ROLL]  += atan2f(-sensors.accel500Hz[YAXIS], -sensors.accel500Hz[ZAXIS]);
		accAngle[PITCH] += atan2f(sensors.accel500Hz[XAXIS], -sensors.accel500Hz[ZAXIS]);

		//求取欧拉角算数平均值
		accAngleSmooth[ROLL ] = accAngle[ROLL ] / (float)initLoops;
		accAngleSmooth[PITCH] = accAngle[PITCH] / (float)initLoops;

		delay(2);
	}

	//得到当前方位 ,初始化一次，不要振动云台，因为这里只用了加速度数据计算欧拉角（加速度数据是长期可信的），但是加速度计对
	//振动很敏感，所以为了减小误差，初始化方位的时候不要振动云台。
	sensors.evvgcCFAttitude500Hz[PITCH] = accAngleSmooth[PITCH];
	sensors.evvgcCFAttitude500Hz[ROLL ] = accAngleSmooth[ROLL ];
	sensors.evvgcCFAttitude500Hz[YAW  ] = 0.0f;
}

///////////////////////////////////////////////////////////////////////////////
//此函数是方位估计的核心函数
void getOrientation(float *smoothAcc, float *orient, float *accData, float *gyroData, float dt)
{
	float accAngle[3];
	float gyroRate[3];

	//通过使用atan2f函数计算加速度数据得到欧拉角 滚转角和 俯仰角。
	accAngle[ROLL ] = atan2f(-accData[YAXIS], -accData[ZAXIS]);
	accAngle[PITCH] = atan2f(accData[XAXIS], -accData[ZAXIS]);

	//其中 smoothAcc 是通过加速度数据经过atan2f函数计算得来的欧拉角，并且进行了一阶滞后滤波
	//（此滤波算法也属于低通滤波的一种），优点： 对周期性干扰具有良好的抑制作用 适用于波动频率较高的场合,
	// 缺点： 相位滞后，灵敏度低 滞后程度取决于a值大小， 不能消除滤波频率高于采样频率的1/2的干扰信号,代码中a的值是99.0f
	smoothAcc[ROLL]  = ((smoothAcc[ROLL ] * 99.0f) + accAngle[ROLL ]) / 100.0f;
	smoothAcc[PITCH] = ((smoothAcc[PITCH] * 99.0f) + accAngle[PITCH]) / 100.0f;

	gyroRate[PITCH] =  gyroData[PITCH];
	//通过互补滤波来融合根据加速度和陀螺仪计算出来的角度，orient[PITCH]是上次融合后的角度，gyroRate[PITCH] * dt是根据陀螺仪
	//数据计算得到的角度（角速度*持续时间结果就是弧度，弧度和角度很容易的可以相互转换），为什么要进行数据融合？
	//答：加速度计和陀螺仪都能计算出姿态，但为何要对它们融合，是因为加速度计对振动之类的扰动很敏感，但长期数据计算出的姿态可信，
	//而陀螺仪虽然对振动这些不敏感，但长期使用陀螺仪会出现漂移，因此我们要进行互补，短期相信陀螺仪，长期相信加速度计.
	//先通过加速度计得到的角度减去上一次融合后的角度然后乘以一个比例系数，这个比例系数越小，融合的加速度计的数据比重越小，
	//短期相信陀螺仪，所以陀螺仪的比重这里是1，长期相信加速度计，加速度计的数据用来修正陀螺仪的漂移产生的误差，
	//这样对陀螺仪的漂移进行了修正，有效地抑制了加速度计和陀螺仪各自单独工作时候的偏差.
	orient[PITCH]   = (orient[PITCH] + gyroRate[PITCH] * dt) + 0.0002f * (smoothAcc[PITCH] - orient[PITCH]);

	//可以用正弦或余弦和x轴单独算出角度,因为我们知道重力的大小
	//但是IMU单元必须是静止水平状态
	gyroRate[ROLL]  =  gyroData[ROLL] * cosf(fabsf(orient[PITCH])) + gyroData[YAW] * sinf(orient[PITCH]);

	//通过互补滤波来融合根据加速度和陀螺仪计算出来的角度，orient[PITCH]是上次融合后的角度，gyroRate[PITCH] * dt是根据陀螺仪
	//数据计算得到的角度（角速度*持续时间结果就是弧度，弧度和角度很容易的可以相互转换），为什么要进行数据融合？
	//答：加速度计和陀螺仪都能计算出姿态，但为何要对它们融合，是因为加速度计对振动之类的扰动很敏感，但长期数据计算出的姿态可信，
	//而陀螺仪虽然对振动这些不敏感，但长期使用陀螺仪会出现漂移，因此我们要进行互补，短期相信陀螺仪，长期相信加速度计.
	//先通过加速度计得到的角度减去上一次融合后的角度然后乘以一个比例系数，这个比例系数越小，融合的加速度计的数据比重越小，
	//短期相信陀螺仪，所以陀螺仪的比重这里是1，长期相信加速度计，加速度计的数据用来修正陀螺仪的漂移产生的误差，
	//这样对陀螺仪的漂移进行了修正，有效地抑制了加速度计和陀螺仪各自单独工作时候的偏差.
	orient[ROLL]    = (orient[ROLL] + gyroRate[ROLL] * dt) + 0.0002f * (smoothAcc[ROLL] - orient[ROLL]);

	//可以用正弦或余弦和x轴单独算出角度,因为我们知道重力的大小
	//但是IMU单元必须是静止水平状态
	gyroRate[YAW]   =  gyroData[YAW] * cosf(fabsf(orient[PITCH])) - gyroData[ROLL] * sinf(orient[PITCH]);

	orient[YAW]     = (orient[YAW] + gyroRate[YAW] * dt);//对陀螺仪进行积分得到偏航角YAW
}

///////////////////////////////////////////////////////////////////////////////
