/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2019,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		headfile
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		查看doc内version文件 版本说明
 * @Software 		IAR 8.3 or MDK 5.28
 * @Target core		NXP RT1064DVL6A
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2019-04-30
 ********************************************************************************************************************/

#ifndef _headfile_h
#define _headfile_h

#include <stdint.h>
#include "fsl_common.h"

#include "fsl_debug_console.h"
#include "fsl_iomuxc.h"
#include "fsl_pit.h"

#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_cache.h"
#include "common.h"
#include "zf_vector.h"

//------文件系统相关头文件
#include "ff.h"
#include "diskio.h"
#include "fsl_sd_disk.h"

#include "SEEKFREE_PRINTF.h"

//------逐飞科技单片机外设驱动头文件
#include "zf_gpio.h"
#include "zf_iomuxc.h"
#include "zf_pit.h"
#include "zf_pwm.h"
#include "zf_uart.h"
#include "zf_spi.h"
#include "zf_systick.h"
#include "zf_qtimer.h"
#include "zf_adc.h"
#include "zf_iic.h"
#include "zf_flash.h"
#include "zf_camera.h"
#include "zf_csi.h"
#include "zf_rom_api.h"
#include "zf_usb_cdc.h"
#include "zf_sdcard.h"

//------RTT头文件
#include "rtthread.h"

//------逐飞科技产品驱动头文件
#include "SEEKFREE_FONT.h"
#include "SEEKFREE_18TFT.h"
#include "SEEKFREE_IPS114_SPI.h"
#include "SEEKFREE_IPS200_PARALLEL8.h"
#include "SEEKFREE_IIC.h"
#include "SEEKFREE_VIRSCO.h"
#include "SEEKFREE_FUN.h"
#include "SEEKFREE_MPU6050.h"
#include "SEEKFREE_OLED.h"
#include "SEEKFREE_NRF24L01.h"
#include "SEEKFREE_MMA8451.h"
#include "SEEKFREE_L3G4200D.h"
#include "SEEKFREE_ICM20602.h"
#include "SEEKFREE_MT9V03X.h"
#include "SEEKFREE_WIRELESS.h"
#include "SEEKFREE_MT9V03X_CSI.h"
#include "SEEKFREE_W25QXXJV.h"
#include "SEEKFREE_SCC8660_CSI.h"
#include "SEEKFREE_SCC8660.h"

#endif
// * -------------------------------- 变换矩阵 -------------------------------- *//
#ifndef GET_TRANSFORM_MATRIX_H
#define GET_TRANSFORM_MATRIX_H

/* Include Files */
#include "get_transform_matrix.h"
#include "rtwtypes.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /* Function Declarations */
    extern void get_transform_matrix(const float corner_coordinates[8],
                                     float transform_matrix[9]);

    extern void get_transform_matrix_initialize(void);

    extern void get_transform_matrix_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
// * -------------------------------- 变换矩阵 -------------------------------- *//

// **************************** 用户宏定义 ****************************
// * -------------------------------- 更换母板前 -------------------------------- *//
// #define FL_PWM (PWM1_MODULE3_CHA_D0)
// #define FR_PWM (PWM1_MODULE3_CHB_D1)
// #define RL_PWM (PWM2_MODULE2_CHB_C11)
// #define RR_PWM (PWM2_MODULE3_CHA_B9)

// #define FL_DIR (D2)
// #define FR_DIR (D3)
// #define RL_DIR (B12)
// #define RR_DIR (B13)
// * -------------------------------- 更换母板前 -------------------------------- *//

// * -------------------------------- 更换母板后 -------------------------------- *//
#define FL_PWM (PWM1_MODULE3_CHB_D1)
#define FR_PWM (PWM2_MODULE3_CHB_D3)
#define RL_PWM (PWM1_MODULE0_CHA_D12)
#define RR_PWM (PWM1_MODULE1_CHA_D14)

#define FL_DIR (D2)
#define FR_DIR (D0)
#define RL_DIR (D13)
#define RR_DIR (D15)
// * -------------------------------- 更换母板后 -------------------------------- *//

#define duty_convert(x) (x / 100.0 * PWM_DUTY_MAX) // x为期望的占空比, 该函数将期望占空比直接转换为 pwm_duty 函数接受的参数

#define encoder_gear_count (45.0)
#define encoder_line_count (1024.0)
#define wheel_week_length (19.163) // 轮子周长, 单位为厘米
#define wheel_gear_count (104.0)
#define wheel_r (2.45)
#define L_ (12.5) // 半长
#define l_ (10)   // 半宽

#define CORE_LED B9

// * -------------------------------- 更换母板前 -------------------------------- *//
// #define FL_ENCODER_A QTIMER1_TIMER2_C2
// #define FL_ENCODER_B QTIMER1_TIMER3_C24

// #define FR_ENCODER_A QTIMER1_TIMER0_C0
// #define FR_ENCODER_B QTIMER1_TIMER1_C1

// #define RL_ENCODER_A QTIMER3_TIMER0_B16
// #define RL_ENCODER_B QTIMER3_TIMER1_B17

// #define RR_ENCODER_A QTIMER4_TIMER0_C9
// #define RR_ENCODER_B QTIMER4_TIMER1_C10
// * -------------------------------- 更换母板前 -------------------------------- *//

// * -------------------------------- 更换母板后 -------------------------------- *//
#define FL_ENCODER_TIMER QTIMER_3
#define FL_ENCODER_A QTIMER3_TIMER2_B18
#define FL_ENCODER_B QTIMER3_TIMER3_B19

#define FR_ENCODER_TIMER QTIMER_1
#define FR_ENCODER_A QTIMER1_TIMER0_C0
#define FR_ENCODER_B QTIMER1_TIMER1_C1

#define RL_ENCODER_TIMER QTIMER_1
#define RL_ENCODER_A QTIMER1_TIMER3_C24
#define RL_ENCODER_B QTIMER1_TIMER2_C2

#define RR_ENCODER_TIMER QTIMER_2
#define RR_ENCODER_A QTIMER2_TIMER3_C25
#define RR_ENCODER_B QTIMER2_TIMER0_C3

#define ART_Front_UART USART_1
#define ART_Down_UART USART_4

#define ART_BAUD 115200

#define ART_Front_UART_TX UART1_TX_B12 // 朝前看的
#define ART_Down_UART_TX UART4_TX_C16  // 向下看的

#define ART_Front_UART_RX UART1_RX_B13
#define ART_Down_UART_RX UART4_RX_C17

#define Magnet C30
#define Servo_PWM PWM4_MODULE3_CHA_C31

#define LED10 C6

// C30电磁铁0/1 C31 舵机PWM
// * -------------------------------- 更换母板后 -------------------------------- *//

#define encoder_sample_time_ms (50.0) // ! 在这个数是50的时候pid好像不错
// **************************** 用户宏定义 ****************************

// **************************** 用户结构体类型定义 ****************************

typedef struct PIDController
{

    /* Controller gains */
    float Kp;
    float Ki;
    float Kd;

    /* Derivative low-pass filter time constant */
    float tau;

    /* Output limits */
    float limMin;
    float limMax;

    /* Integrator limits */
    float limMinInt;
    float limMaxInt;

    /* Sample time (in seconds) */
    float T;

    /* Controller "memory" */
    float integrator;
    float prevError; /* Required for integrator */
    float differentiator;
    float prevMeasurement; /* Required for differentiator */

    /* Controller output */
    float out;
    float prev_out;

} PIDController;

// **************************** 用户结构体类型定义 ****************************