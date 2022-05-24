#include "zf_iic.h"
#include "zf_systick.h"
#include "SEEKFREE_IIC.h"
#include "gy_85.h"

unsigned char BUF[6];
INT16_XYZ A_value, H_value, T_value;
INT16_XYZ T_offset;
double Angle;
float imu_acc_x = 0, imu_acc_y = 0, imu_acc_z = 0;

//*******ADXL345*********************************
void read_ADXL345(void)
{
	simiic_read_regs(ADXL345_Addr, 0x32, BUF, 6, SIMIIC); //先输出低位再输出高位

	A_value.X = (BUF[1] << 8) + BUF[0]; //合成数据
	A_value.Y = (BUF[3] << 8) + BUF[2]; //合成数据
	A_value.Z = (BUF[5] << 8) + BUF[4]; //合成数据
	imu_acc_x = A_value.X * 0.004 * SENSORS_GRAVITY_EARTH;
	imu_acc_y = A_value.Y * 0.004 * SENSORS_GRAVITY_EARTH;
	imu_acc_z = A_value.Z * 0.004 * SENSORS_GRAVITY_EARTH;
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

/***以下函数放于定时器中2ms执行一次***/
void Angle_get(void)
{
	static int angle_count = 0; //刚开始，单独使用加速度标志
	double angle_ratio = 0;		//加速度比值
	static float Pitch = 0;
	static float Angle_acc_last;
	/*******（滤波过程略)*/
	float dt = 0.002; // Gy 2ms时间积分系数
	double Angle_acc;
	//		Anglefiltering();//入口滤波，算数平均
	/***以下为刚开始时的加速度角度单独处理***/
	Angle_acc_last = Angle_acc;
	read_ADXL345();
	angle_ratio = ((double)A_value.X) / (A_value.Z + 0.1);
	Angle_acc = (float)atan(angle_ratio) * 57.29578049; //加速度计得到的角

	if (angle_count == 0) //低通限幅滤波
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
	T_value.Z -= T_offset.Z;
	T_value.Z = T_value.Z / 14.375; //换算
	Pitch += T_value.Z * dt;

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
