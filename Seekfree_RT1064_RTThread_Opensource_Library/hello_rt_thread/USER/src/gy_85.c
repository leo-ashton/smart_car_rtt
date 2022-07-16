#include "zf_iic.h"
#include "zf_systick.h"
#include "SEEKFREE_IIC.h"
#include "gy_85.h"
// #include "kalman.h"
#include "math.h"
#include "assert.h"
#include "arm_math.h"

#define jiaozheng_count (50)

unsigned char BUF[6];
INT16_XYZ A_value, H_value, T_value; // 读到的三个传感器的值，存放计算得到坐标
INT16_XYZ T_offset;
double Angle;
float_XYZ Dis, V_value, Dis2;
float imu_acc_x = 0, imu_acc_y = 0, imu_acc_z = 0;
float imu_acc_x_last = 0, imu_acc_y_last = 0;																	  //记录上一次
float imu_acc_x_first[110] = {0}, imu_acc_y_first[110] = {0}, imu_acc_x_2nd[150] = {0}, imu_acc_y_2nd[150] = {0}; //
float A_average_x1 = 0, A_average_y1 = 0, A_average_x2 = 0, A_average_y2 = 0;
int count_flag = 0;
float V_value_x_last = 0, V_value_y_last = 0;
float ax = 0, ay = 0;
int jiaozheng = 0;
int flag = 0;
extern float v_fl, v_fr, v_rl, v_rr, omega_z;
float v_fl_filtered = 0, v_fr_filtered = 0, v_rl_filtered = 0, v_rr_filtered = 0;

//卡尔曼滤波设初值
KFP KFP_imu_acc_x = {0.02, 0, 0, 0, 0.001, 0.6};
KFP KFP_imu_acc_y = {0.02, 0, 0, 0, 0.001, 0.6};
KFP KFP_encoder_fl = {0.02, 0, 0, 0, 0.01, 0.6};
KFP KFP_encoder_fr = {0.02, 0, 0, 0, 0.01, 0.6};
KFP KFP_encoder_rl = {0.02, 0, 0, 0, 0.01, 0.6};
KFP KFP_encoder_rr = {0.02, 0, 0, 0, 0.01, 0.6};
KFP KFP_V_value_x = {0.02, 0, 0, 0, 0.001, 0.6};
KFP KFP_V_value_y = {0.02, 0, 0, 0, 0.001, 0.6};
KFP KFP_D_value_x = {0.02, 0, 0, 0, 0.001, 0.6};
KFP KFP_D_value_y = {0.02, 0, 0, 0, 0.001, 0.6};
//*******ADXL345*********************************
void accelerater_init()
{
	// read_ADXL345();
	// // 读取前
	// // imu_acc_x=kalmanFilter(&KFP_imu_acc_x2,imu_acc_x);
	// // imu_acc_y=kalmanFilter(&KFP_imu_acc_y2,imu_acc_y);
	// if (count_flag < 150 && count_flag > 39) //记录第40次到150次的值，第一次修正
	// {
	// 	imu_acc_x_first[count_flag - 40] = imu_acc_x;
	// 	imu_acc_y_first[count_flag - 40] = imu_acc_y;
	// 	count_flag += 1;
	// }
	// else if (count_flag == 150)
	// {
	// 	// 求平均
	// 	A_average_x1 = aver110(imu_acc_x_first);
	// 	A_average_y1 = aver110(imu_acc_y_first);
	// 	count_flag += 1;
	// }
	// else if (count_flag < 301) // 第二次修正,151到301次
	// {
	// 	//再读取第一次修正后的值，进行第二次修正
	// 	imu_acc_x -= A_average_x1;
	// 	imu_acc_y -= A_average_y1;
	// 	imu_acc_x_2nd[count_flag - 151] = imu_acc_x;
	// 	imu_acc_y_2nd[count_flag - 151] = imu_acc_y;
	// 	count_flag += 1;
	// }
	// else if (count_flag == 301)
	// {
	// 	//再计算平均值，第二次的平均值
	// 	A_average_x2 = aver150(imu_acc_x_2nd);
	// 	A_average_y2 = aver150(imu_acc_y_2nd);
	// 	count_flag += 1;
	// }
}
void read_ADXL345(void)
{
	simiic_read_regs(ADXL345_Addr, 0x32, BUF, 6, SIMIIC); //先输出低位再输出高位

	A_value.X = (BUF[1] << 8) + BUF[0]; //合成数据
	A_value.Y = (BUF[3] << 8) + BUF[2]; //合成数据
	A_value.Z = (BUF[5] << 8) + BUF[4]; //合成数据
	imu_acc_x = A_value.X * 0.004 * SENSORS_GRAVITY_EARTH;
	imu_acc_y = A_value.Y * 0.004 * SENSORS_GRAVITY_EARTH;
	imu_acc_z = A_value.Z * 0.004 * SENSORS_GRAVITY_EARTH;
	imu_acc_x = kalmanFilter(&KFP_imu_acc_x, imu_acc_x);
	imu_acc_y = kalmanFilter(&KFP_imu_acc_y, imu_acc_y);
}

void Init_ADXL345(void)
{
	systick_delay_ms(100);						//上电延时
	simiic_write_reg(ADXL345_Addr, 0x31, 0x0B); //测量范围,正负16g，13位模式
	simiic_write_reg(ADXL345_Addr, 0x2C, 0x0e); //速率设定为100hz 参考pdf13页
	simiic_write_reg(ADXL345_Addr, 0x2D, 0x08); //选择电源模式   参考pdf24页
	simiic_write_reg(ADXL345_Addr, 0x2E, 0x80); //使能 DATA_READY 中断
}

//*****HMC58831**********************
float read_hmc5883l(void)
{
	float Current_Angle;
	simiic_read_regs(HMC5883L_Addr, 0x03, BUF, 6, SIMIIC); //高位在前低位在后
	H_value.X = (BUF[0] << 8) + BUF[1];					   //合成数据
	H_value.Z = (BUF[2] << 8) + BUF[3];
	H_value.Y = (BUF[4] << 8) + BUF[5];
	Current_Angle = (atan2(H_value.Y, H_value.X) * (180 / 3.14159265) + 180);
	return Current_Angle;
}

void Init_HMC5883L(void)
{
	systick_delay_ms(100);

	unsigned char cdata[3] = {0x70, 0x20, 0X00};
	simiic_write_reg(HMC5883L_Addr, HMC5883l_CONFIG_A, *cdata);			//设置数据输出速率
	simiic_write_reg(HMC5883L_Addr, HMC5883l_CONFIG_B, *(cdata + 1));	//设置增益，±1.3Ga
	simiic_write_reg(HMC5883L_Addr, HMC5883l_MODECONFIG, *(cdata + 2)); //设置测量模式，连续测量模式
}

//*****ITG3205****************************************
void Init_ITG3205(void)
{
	simiic_write_reg(ITG3205_Addr, PWR_M, 0x80); //
	simiic_write_reg(ITG3205_Addr, SMPL, 0x07);	 //
	simiic_write_reg(ITG3205_Addr, DLPF, 0x1E);	 //±2000°
	simiic_write_reg(ITG3205_Addr, INT_C, 0x00); //
	simiic_write_reg(ITG3205_Addr, PWR_M, 0x00); //
}

void read_ITG3205(void)
{
	simiic_read_regs(ITG3205_Addr, 0x1D, BUF, 6, SIMIIC); //高位在前低位在后
	T_value.X = (BUF[0] << 8) | BUF[1];
	//	 *imu_gyro_x = T_value.X/14.375;

	T_value.Y = (BUF[2] << 8) | BUF[3];
	//  *imu_gyro_y = T_value.Y/14.375;				  //读取计算Y轴数据

	T_value.Z = (BUF[4] << 8) | BUF[5];
	//  *imu_gyro_z = T_value.Z/14.375;					       //读取计算Z轴数据
}

/******************************************************************************
计算加速度或角速度的零漂值
*参  数：value： 原始数据
*        offset：校准后的零偏值
*        sensivity：加速度计的灵敏度
*返回值：1校准完成 0校准未完成
*备  注：无
*******************************************************************************/
uint8_t ITG3205_OffSet(INT16_XYZ value, INT16_XYZ *offset, uint16_t sensivity)
{
	static int32_t tempgx = 0, tempgy = 0, tempgz = 0;
	static uint16_t cnt_a = 0; //使用static修饰的局部变量，表明次变量具有静态存储周期，也就是说该函数执行完后不释放内存
	if (cnt_a == 0)
	{
		value.X = 0;
		value.Y = 0;
		value.Z = 0;
		tempgx = 0;
		tempgy = 0;
		tempgz = 0;
		cnt_a = 1;
		sensivity = 0;
		offset->X = 0;
		offset->Y = 0;
		offset->Z = 0;
	}
	tempgx += value.X;
	tempgy += value.Y;
	tempgz += value.Z - sensivity; //加速度计校准 sensivity 等于 MPU9250初始化时设置的灵敏度值（8196LSB/g）;陀螺仪校准 sensivity = 0；
	if (cnt_a == 200)			   // 200个数值求平均
	{
		offset->X = tempgx / cnt_a;
		offset->Y = tempgy / cnt_a;
		offset->Z = tempgz / cnt_a;
		cnt_a = 0;
		return 1;
	}
	cnt_a++;
	return 0;
}

//****进行去零漂处理*************
void ITG3205_Offset_Calc(void)
{
	int8 flag; //校准采样是否完成
	do
	{
		read_ITG3205();
		flag = ITG3205_OffSet(T_value, &T_offset, 0);
		systick_delay_ms(2);
	} while (flag == 0);
	return;
}

/***以下函数放于定时器中2ms执行一次，***/ // A 加速度（得到三个轴上加速度的大小（重力加速度））   H 地磁场加速度（得到磁场在x和y轴上的大小，可以得到当前小车的转向，需要先转一圈进行读数，然后再读表）   ITG陀螺仪（得到三个轴的角加速度）
void Angle_get(void)
{
	static int angle_count = 0; //刚开始，单独使用加速度标志
	double angle_ratio = 0;		//加速度比值
	static float Pitch = 0;
	static float Angle_acc_last;
	static float T_value_last = 0;
	static float A_count = 1;
	/*******（滤波过程略)*/
	float dt = 0.002; // Gy 2ms时间积分系数
	double Angle_acc;
	//		Anglefiltering();//入口滤波，算数平均
	/***以下为刚开始时的加速度角度单独处理***/
	Angle_acc_last = Angle_acc;
	read_ADXL345();
	angle_ratio = ((double)A_value.X) / (A_value.Z + 0.1);
	Angle_acc = (float)atan(angle_ratio) * 57.29578049; //加速度计得到的角
	if (angle_count == 0)								//低通限幅滤波，第一次运行程序
	{
		Angle_acc_last = Angle_acc;
	}
	Angle_acc = 0.5 * Angle_acc + 0.5 * Angle_acc_last;
	if (Angle_acc > 89)
		Angle_acc = 89;
	if (Angle_acc < -89)
		Angle_acc = -89;
	/***陀螺仪角度读取，对Z轴角速度进行积分*******************************/
	read_ITG3205();
	if (A_count)
	{
		T_value_last = T_value.Z;
		A_count = 0;
	}
	T_value.Z -= T_offset.Z;
	T_value.Z = T_value.Z / 14.375; //换算
	Pitch += T_value.Z * dt - fabs(T_value.Z - T_value_last) / 2 * dt;
	T_value_last = T_value.Z;
	if (angle_count < 300) //刚开始运行的0.6s时，取加速度轴得到的角度
	{
		Angle = Angle_acc; //加速度得到的角度
		angle_count++;
	}
	else
	{
		Angle = Pitch; //融合后的结果
	}
}
//
void init_Dis(void)
{
	V_value.X = V_value.Y = 0;
	Dis.X = Dis.Y = 0;
}

float kalmanFilter(KFP *kfp, float input)
{
	//预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
	kfp->Now_P = kfp->LastP + kfp->Q;
	//卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
	kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
	//更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
	kfp->out = kfp->out + kfp->Kg * (input - kfp->out); //因为这一次的预测值就是上一次的输出值
	//更新协方差方程: 本次的系统协方差付给 kfp->LastP 威下一次运算准备。
	kfp->LastP = (1 - kfp->Kg) * kfp->Now_P;
	return kfp->out;
}

float aver110(float a[110])
{
	int i;
	float av, s = a[110];
	for (i = 1; i < 110; i++)
		s = s + a[i];
	av = s / 110;
	return av;
}

float aver150(float a[150])
{
	int i;
	float av, s = a[150];
	for (i = 1; i < 150; i++)
		s = s + a[i];
	av = s / 150;
	return av;
}

void V_get(void)
{
	float dt1 = 0.01;
	static int i = 0, A_sum_x = 0;

	read_ADXL345();
	// imu_acc_x=kalmanFilter(&KFP_imu_acc_x2,imu_acc_x);
	// imu_acc_y=kalmanFilter(&KFP_imu_acc_y2,imu_acc_y);
	if (count_flag < 150 && count_flag > 39) //记录第40次到150次的值，第一次修正
	{
		imu_acc_x_first[count_flag - 40] = imu_acc_x;
		imu_acc_y_first[count_flag - 40] = imu_acc_y;
		count_flag += 1;
	}
	else if (count_flag == 150)
	{
		// 求平均
		A_average_x1 = aver110(imu_acc_x_first);
		A_average_y1 = aver110(imu_acc_y_first);
		count_flag += 1;
	}
	else if (count_flag < 301) // 第二次修正,151到301次
	{
		//再读取第一次修正后的值，进行第二次修正
		imu_acc_x -= A_average_x1;
		imu_acc_y -= A_average_y1;
		imu_acc_x_2nd[count_flag - 151] = imu_acc_x;
		imu_acc_y_2nd[count_flag - 151] = imu_acc_y;
		count_flag += 1;
	}
	else if (count_flag == 301)
	{
		//再计算平均值，第二次的平均值
		A_average_x2 = aver150(imu_acc_x_2nd);
		A_average_y2 = aver150(imu_acc_y_2nd);
		count_flag += 1;
	}
	else // 两次校正之后运行
	{
		imu_acc_x = imu_acc_x - A_average_x1 - A_average_x2;
		imu_acc_y = imu_acc_y - A_average_y1 - A_average_y2;

		if (fabs(imu_acc_x) <= 0.2) // 认为加速度为零
		{
			V_value.X += 0; // 得到速度，一阶积分
			imu_acc_x_last = 0;
			// V_value.X = kalmanFilter(&KFP_V_value_x, V_value.X);
			V_value.Y += 0 * dt1 - fabs(0 - imu_acc_y_last) / 2 * dt1; // 得到速度，一阶积分
			imu_acc_y_last = 0;
			// V_value.Y = kalmanFilter(&KFP_V_value_y, V_value.Y);
		}
		else
		{
			V_value.X = V_value.X + imu_acc_x * dt1 - fabs(imu_acc_x - imu_acc_x_last) / 2 * dt1; // 得到速度，一阶积分
			imu_acc_x_last = imu_acc_x;
			V_value.X = kalmanFilter(&KFP_V_value_x, V_value.X);
			V_value.Y = V_value.Y + imu_acc_y * dt1 - fabs(imu_acc_y - imu_acc_y_last) / 2 * dt1; // 得到速度，一阶积分
			imu_acc_y_last = imu_acc_y;
			V_value.Y = kalmanFilter(&KFP_V_value_y, V_value.Y);
		}
		jiaozheng = jiaozheng + 1;
		flag = 0;
		// 矫正时间=rt_thread_mdelay*jiaozheng_count（ms）
		if (jiaozheng == jiaozheng_count) // 强制为零
		{
			jiaozheng = 0;

			if (fabs(imu_acc_x) <= 0.2)
			{
				V_value.X = 0;
				imu_acc_x_last = 0;
				flag = 1;
				// assert(0);
			}
			if (fabs(imu_acc_y) <= 0.1)
			{
				V_value.Y = 0;
				imu_acc_y_last = 0;
			}
		}
	}
}

void Dis_get(void)
{
	// static float dt2 = 0.05;
	// Dis.X += V_value.X * dt2 - (V_value.X - V_value_x_last) * dt2;
	// V_value_x_last = V_value.X;
	// Dis.Y += V_value.Y * dt2 - (V_value.Y - V_value_y_last) * dt2;
	// V_value_y_last = V_value.Y;
	// Dis.X = kalmanFilter(&KFP_D_value_x, Dis.X);
	// Dis.Y = kalmanFilter(&KFP_D_value_y, Dis.Y);
	//滤波
	// v_fl_filtered = kalmanFilter(&KFP_encoder_fl, v_fl);  //前左
	// v_fr_filtered = -kalmanFilter(&KFP_encoder_fr, v_fr); //前右
	// v_rl_filtered = kalmanFilter(&KFP_encoder_rl, v_rl);  //后左
	// v_rr_filtered = -kalmanFilter(&KFP_encoder_rr, v_rr); //后右

	v_fl_filtered = v_fl; //前左
	v_fr_filtered = v_fr; //前右
	v_rl_filtered = v_rl; //后左
	v_rr_filtered = v_rr; //后右

	// 计算位移
	// Dis2.X += 1.414 / 2 * (v_fl_filtered + v_fr_filtered + v_rl_filtered + v_rr_filtered) * dt2;
	// Dis2.Y += 1.414 / 2 * (-v_fl_filtered + v_fr_filtered + v_rl_filtered - v_rr_filtered) * dt2;

	// Dis2.X += 1.414 / 2 * (v_fl + v_fr + v_rl + v_rr) * dt2;
	// Dis2.Y += 1.414 / 2 * (-v_fl + v_fr + v_rl - v_rr) * dt2;

	// theta += omega_z * dt2;
}