#include "headfile.h"
#include "math.h"
#include "gy_85.h"
#include "user_debug.h"
#include <rtthread.h>
#include "arm_math.h"
#include <string.h>
// ! 为了进行系统辨识，修改了encoder_sample_time_ms的值
// 舵机搬图片抬起为8 无图片抬起15 完全放下为5
// * -------------------------------- 分割线 -------------------------------- *//

// * 对电机系统辨识时,调整零极点的个数, 一般使用二阶震荡模型, 即 2 极点 0 零点.
// TODO 调整编码器采样时间后, 记得修改速度公式里的脉冲读取时间
// * RT_TICK_PER_SECOND = 1000，于是 1 OSTick = 1 ms
// * 左后轮调好了, 速度注意为正 右后为正 右前为正 左前为正
// 之前震荡的原因是什么? 把 behavior 调整到 robust 一侧就好了

// * -------------------------------- 需要在串口打印的数据 -------------------------------- *//
uint8 PRINT_SPEED_ENABLED = 1;
// * -------------------------------- 需要在串口打印的数据 -------------------------------- *//

// * -------------------------------- 一些设置 -------------------------------- *//
// ! PID 默认关闭
uint8 SPEED_PID_ENABLED = 0;
uint8 POSITION_PID_ENABLED = 0;
uint8 ANGLE_SPEED_PID_ENABLED = 0;
uint8 ANGLE_PID_ENABLED = 0;
#define MAX_SPEED (550 * 0.6)
// * -------------------------------- 一些设置 -------------------------------- *//

// **************************** 变量定义 ****************************
static rt_sem_t encoder_fl_sem = RT_NULL; // 创建指向信号量的指针
static rt_sem_t encoder_fr_sem = RT_NULL; // 创建指向信号量的指针
static rt_sem_t encoder_rl_sem = RT_NULL; // 创建指向信号量的指针
static rt_sem_t encoder_rr_sem = RT_NULL; // 创建指向信号量的指针

static int32 v_fl_expect = 0, v_fr_expect = 0, v_rl_expect = 0, v_rr_expect = 0;
float x_expect = 0, y_expect = 0;
float omega_z_expected = 0;
int32 encoder_fl_val = 0, encoder_fr_val = 0, encoder_rl_val = 0, encoder_rr_val = 0;
float v_fl, v_fr, v_rl, v_rr, omega_z = 0; // ! 均为测量值
float theta = 0;
float theta_expected = 0;

extern double Angle;
extern float_XYZ Dis, Dis2;
extern float imu_acc_x, imu_acc_y, imu_acc_z;
extern float_XYZ V_value, A_value, Dis;
extern float A_average_x1, A_average_x2;
extern int count_flag;
extern float V_value_x_last;
extern float ax, ay;
extern int flag;

extern float v_fl_filtered, v_fr_filtered, v_rl_filtered, v_rr_filtered;

uint8 allow_receive_revise_message = 0;
// uint8
// v_fl_filtered = kalmanFilter(&KFP_encoder_fl, v_fl);  //前左
// v_fr_filtered = -kalmanFilter(&KFP_encoder_fr, v_fr); //前右
// v_rl_filtered = kalmanFilter(&KFP_encoder_rl, v_rl);  //后左
// v_rr_filtered = -kalmanFilter(&KFP_encoder_rr, v_rr); //后右

// extern float v_fl_filtered,v_fr_filtered,v_rl_filtered,v_rr_filtered;

// **************************** 变量定义 ****************************

// **************************** 函数定义 ****************************
void devices_init();
int create_main_threads_group(void);
int create_main_timer(void);
void read_encoder_thread_entry(void *parameter);
void read_GY_85_thread_entry(void *parameter);
void receive_message_thread_entry(void *parameter);
void readV__D(void *parameter);
static void readV__D_for_timer(void *parameter);

void PIDController_Init(PIDController *pid, char *wheel_axis, char *type);
void motor_control_entry(void *parameter);
float PIDController_weight_update(PIDController *pid, float setpoint, float measurement, char *type);

// * 跟调试有关的函数
int create_system_identification_threads_group(void);

static void speed_set(int argc, char **argv);
static void position_set(int argc, char **argv);
static void omega_z_set(int argc, char **argv);
static void dir_set(int argc, char **argv);

void minimum_encoder_printer_entry(void *parameter);
static void duty_set(int argc, char **argv);
static void enable_pid(int argc, char **argv);

void get_true_coordinates(float new_height, float new_width, float *corner_x, float *corner_y, float *raw_coordinates_x, float *raw_coordinates_y, arm_matrix_instance_f32 *result); // 输入角点与校正前圆心坐标，返回校正后各目标点坐标
static void test_get_true_coordinates(int argc, char **argv);

void direction_control(char direction, float speed);
static void direction_control_m(int argc, char **argv);
static void test_matrix_function();
static void test_move(int argc, char **argv);
// **************************** 函数定义 ****************************

// **************************** msh 初始化 ****************************
MSH_CMD_EXPORT(speed_set, "speed_set sample - speed_set fl 100");
MSH_CMD_EXPORT(dir_set, "dir_set sample - dir_set fl 1");
MSH_CMD_EXPORT(duty_set, "sweep_pwm_duty sample - duty_set fl 50");
MSH_CMD_EXPORT(enable_pid, "dir_set sample - dir_set fl 1");
MSH_CMD_EXPORT(direction_control_m, "direction_control sample - direction_control_m w 50");
MSH_CMD_EXPORT(test_get_true_coordinates, "Dont need parameters");
MSH_CMD_EXPORT(test_matrix_function, "Dont need parameters");
MSH_CMD_EXPORT(position_set, "position_set x 10");
MSH_CMD_EXPORT(test_move, "Dont need parameters");
MSH_CMD_EXPORT(omega_z_set, "omega_z_set 10");

// **************************** msh 初始化 ****************************

int main(void)
{
    //此处编写用户代码(例如：外设初始化代码等)
    devices_init();

    // * --------------------------------信号量初始化--------------------------------
    encoder_fl_sem = rt_sem_create("encoder_fl", 0, RT_IPC_FLAG_PRIO);
    encoder_fr_sem = rt_sem_create("encoder_fr", 0, RT_IPC_FLAG_PRIO);
    encoder_rl_sem = rt_sem_create("encoder_rl", 0, RT_IPC_FLAG_PRIO);
    encoder_rr_sem = rt_sem_create("encoder_rr", 0, RT_IPC_FLAG_PRIO);
    // * --------------------------------信号量初始化--------------------------------

    // * --------------------------------线程初始化--------------------------------
    // ! 启动你需要的线程
    create_main_threads_group();
    // create_system_identification_threads_group();
    // * --------------------------------线程初始化--------------------------------

    // * -------------------------------- 软件定时器初始化 -------------------------------- *//
    // create_main_timer();
    // * -------------------------------- 软件定时器初始化 -------------------------------- *//

    EnableGlobalIRQ(0);
    while (1)
    {
        //此处编写需要循环执行的代码
        gpio_toggle(CORE_LED);
        // rt_kprintf("%ld,%ld,%ld,%ld\n", (int32)(imu_acc_x * 100), (int32)(V_value.X * 100), (int32)(Dis.X * 100), (int32)(flag * 100));
        rt_thread_mdelay(100);
    }
}

void devices_init()
{
    // 完成硬件的初始化.
    // * --------------------------------GPIO 初始化--------------------------------
    gpio_init(CORE_LED, GPO, 0, GPIO_PIN_CONFIG); // LED
    gpio_init(LED10, GPO, 0, GPIO_PIN_CONFIG);    // LED

    gpio_init(FL_DIR, GPO, 0, GPIO_PIN_CONFIG);
    gpio_init(FR_DIR, GPO, 0, GPIO_PIN_CONFIG);
    gpio_init(RL_DIR, GPO, 0, GPIO_PIN_CONFIG);
    gpio_init(RR_DIR, GPO, 0, GPIO_PIN_CONFIG);
    gpio_init(Magnet, GPO, 0, GPIO_PIN_CONFIG); // 电磁铁

    // B29 DIR B25 PWM
    // B28 DIR B24 PWM
    // * --------------------------------GPIO 初始化--------------------------------

    // * --------------------------------编码器初始化--------------------------------
    qtimer_quad_init(FR_ENCODER_TIMER, FR_ENCODER_A, FR_ENCODER_B);
    qtimer_quad_init(FL_ENCODER_TIMER, FL_ENCODER_A, FL_ENCODER_B);

    // * 原始引脚
    // qtimer_quad_init(QTIMER_2, QTIMER2_TIMER0_C3, QTIMER2_TIMER3_C25);
    // qtimer_quad_init(QTIMER_3, QTIMER3_TIMER2_B18, QTIMER3_TIMER3_B19);
    // * 原始引脚

    qtimer_quad_init(RL_ENCODER_TIMER, RL_ENCODER_A, RL_ENCODER_B);
    qtimer_quad_init(RR_ENCODER_TIMER, RR_ENCODER_A, RR_ENCODER_B);

    rt_kprintf("QTIMER initialization succeeded.\n");
    // * --------------------------------编码器初始化--------------------------------

    // * -------------------------------- 电机初始化 --------------------------------
    pwm_init(FL_PWM, 50, 0);
    pwm_init(FR_PWM, 50, 0);
    pwm_init(RL_PWM, 50, 0);
    pwm_init(RR_PWM, 50, 0);
    pwm_init(Servo_PWM, 333, 0); // 舵机,推荐频率为333Hz
    rt_kprintf("PWM initialization succeeded.\n");
    // * -------------------------------- 电机初始化 --------------------------------

    // * -------------------------------- GY-85初始化 --------------------------------
    simiic_init();
    Init_HMC5883L();
    Init_ADXL345();
    Init_ITG3205();
    rt_kprintf("GY-85 initialization succeeded.\n");
    // * -------------------------------- GY-85初始化 --------------------------------

    // * -------------------------------- 串口初始化 -------------------------------- *//
    // b12 b13 c17 c16 OpenART通信接口
    uart_init(ART_Front_UART, ART_BAUD, ART_Front_UART_TX, ART_Front_UART_RX);
    uart_init(ART_Down_UART, ART_BAUD, ART_Down_UART_TX, ART_Down_UART_RX);

    // uart_init(USART_1, 115200, UART1_TX_B12, UART1_RX_B13);
    // uart_init(USART_8, 115200, UART4_TX_C16, UART4_RX_C17);

    rt_kprintf("UART initialization succeeded.\n");
    // * -------------------------------- 串口初始化 -------------------------------- *//
}

void read_encoder_thread_entry(void *parameter)
{
    while (1)
    {
        //读取编码器计数值

        encoder_fl_val = qtimer_quad_get(FL_ENCODER_TIMER, FL_ENCODER_A);
        rt_sem_release(encoder_fl_sem); // 告诉别的线程可以使用encoder_fl_val的值了

        encoder_fr_val = qtimer_quad_get(FR_ENCODER_TIMER, FR_ENCODER_A);
        rt_sem_release(encoder_fr_sem);

        // encoder_rl_val = qtimer_quad_get(QTIMER_2, QTIMER2_TIMER0_C3);
        // rt_sem_release(encoder_rl_sem);

        // encoder_rr_val = qtimer_quad_get(QTIMER_3, QTIMER3_TIMER2_B18);
        // rt_sem_release(encoder_rr_sem);

        encoder_rl_val = qtimer_quad_get(RL_ENCODER_TIMER, RL_ENCODER_A);
        rt_sem_release(encoder_rl_sem);

        encoder_rr_val = qtimer_quad_get(RR_ENCODER_TIMER, RR_ENCODER_A);
        rt_sem_release(encoder_rr_sem);

        qtimer_quad_clear(FR_ENCODER_TIMER, FR_ENCODER_A);
        qtimer_quad_clear(FL_ENCODER_TIMER, FL_ENCODER_A);
        qtimer_quad_clear(RL_ENCODER_TIMER, RL_ENCODER_A);
        qtimer_quad_clear(RR_ENCODER_TIMER, RR_ENCODER_A);
        rt_thread_mdelay(encoder_sample_time_ms);
    }
}

int create_main_threads_group(void)
{
    // * 线程初始化函数
    // 线程控制块指针
    rt_thread_t tid;
    // 创建动态线程

    // * 编码器读取线程创建
    tid = rt_thread_create("read_encoder_thread",     // 线程名称
                           read_encoder_thread_entry, // 线程入口函数
                           RT_NULL,                   // 线程参数
                           1024,                      // 1024 个字节的栈空间
                           5,                         // 线程优先级为5，数值越小，优先级越高，0为最高优先级。
                                                      // 可以通过修改rt_config.h中的RT_THREAD_PRIORITY_MAX宏定义(默认值为8)来修改最大支持的优先级
                           5);                        // 时间片为5

    rt_kprintf("create dynamic thread: read_encoder_thread.\n");
    if (tid != RT_NULL) // 线程创建成功
    {
        rt_kprintf("thread: read_encoder_thread dynamic thread create OK.\n");
        rt_thread_startup(tid); // 运行该线程
    }
    else // 线程创建失败
    {
        rt_kprintf("thread: read_encoder_thread dynamic thread create ERROR.\n");
        return 1;
    }
    // * 编码器读取线程创建

    // * PID控制线程创建
    tid = rt_thread_create("motor_control_entry", // 线程名称
                           motor_control_entry,   // 线程入口函数
                           RT_NULL,               // 线程参数
                           1024,                  // 1024 个字节的栈空间
                           5,                     // 线程优先级为5，数值越小，优先级越高，0为最高优先级。
                                                  // 可以通过修改rt_config.h中的RT_THREAD_PRIORITY_MAX宏定义(默认值为8)来修改最大支持的优先级
                           5);                    // 时间片为5
    rt_kprintf("create dynamic thread: pid_motor_control_thread.\n");
    if (tid != RT_NULL) // 线程创建成功
    {
        rt_kprintf("thread: pid_motor_control_thread dynamic thread create OK.\n");
        rt_thread_startup(tid); // 运行该线程
    }
    else // 线程创建失败
    {
        rt_kprintf("thread: pid_motor_control_thread dynamic thread create ERROR.\n");
        return 1;
    }
    // * PID控制线程创建

    // * 传感器读取线程创建
    tid = rt_thread_create("readV__D_thread", // 线程名称
                           readV__D,          // 线程入口函数
                           RT_NULL,           // 线程参数
                           1024,              // 1024 个字节的栈空间
                           5,                 // 线程优先级为5，数值越小，优先级越高，0为最高优先级。
                                              // 可以通过修改rt_config.h中的RT_THREAD_PRIORITY_MAX宏定义(默认值为8)来修改最大支持的优先级
                           5);                // 时间片为5
    rt_kprintf("create dynamic thread: read_GY_85_thread.\n");
    if (tid != RT_NULL) // 线程创建成功
    {
        rt_kprintf("thread: read_GY_85_thread dynamic thread create OK.\n");
        rt_thread_startup(tid); // 运行该线程
    }
    else // 线程创建失败
    {
        rt_kprintf("thread: read_GY_85_thread dynamic thread create ERROR.\n");
        return 1;
    }
    // * 传感器读取线程创建

    // * OpenART通信线程创建
    tid = rt_thread_create("receive_message_thread",     // 线程名称
                           receive_message_thread_entry, // 线程入口函数
                           RT_NULL,                      // 线程参数
                           1024,                         // 1024 个字节的栈空间
                           5,                            // 线程优先级为5，数值越小，优先级越高，0为最高优先级。
                                                         // 可以通过修改rt_config.h中的RT_THREAD_PRIORITY_MAX宏定义(默认值为8)来修改最大支持的优先级
                           5);                           // 时间片为5
    rt_kprintf("create dynamic thread: receive_message_thread.\n");
    if (tid != RT_NULL) // 线程创建成功
    {
        rt_kprintf("thread: receive_message_thread create OK.\n");
        rt_thread_startup(tid); // 运行该线程
    }
    else // 线程创建失败
    {
        rt_kprintf("thread: receive_message_thread create ERROR.\n");
        return 1;
    }
    // * OpenART通信线程创建
    return 0;
}

void motor_control_entry(void *parameter)
{

    // 读取并打印编码器的值

    static rt_err_t result;
    float revised_x_speed = 0, revised_y_speed = 0;
    float revised_fl_speed, revised_fr_speed, revised_rl_speed, revised_rr_speed = 0;
    float revised_fl_duty, revised_fr_duty, revised_rl_duty, revised_rr_duty = 0;
    float revised_omega_z = 0;
    uint8 dir_fl, dir_fr, dir_rl, dir_rr = 0;
    int32 revised_rl_duty_int = 0;
    const float dt2 = 0.05;

    PIDController fl_speed_pid_controller;
    PIDController fr_speed_pid_controller;
    PIDController rl_speed_pid_controller;
    PIDController rr_speed_pid_controller;

    PIDController x_axis_pid_controller;
    PIDController y_axis_pid_controller;

    PIDController angle_speed_pid_controller;
    PIDController angle_pid_controller;

    PIDController_Init(&fl_speed_pid_controller, "fl", "speed");
    PIDController_Init(&fr_speed_pid_controller, "fr", "speed");
    PIDController_Init(&rl_speed_pid_controller, "rl", "speed");
    PIDController_Init(&rr_speed_pid_controller, "rr", "speed");

    PIDController_Init(&x_axis_pid_controller, "x", "position");
    PIDController_Init(&y_axis_pid_controller, "y", "position");

    PIDController_Init(&angle_speed_pid_controller, "x", "omega");
    PIDController_Init(&angle_pid_controller, "x", "theta");

    // v_rl_expect=-150;
    rt_thread_mdelay(500);

    while (1)
    {
        result = rt_sem_take(encoder_fl_sem, RT_WAITING_FOREVER); // 如果rt_sem_take返回RT_EOK,表明编码器的值可以使用了
        result = rt_sem_take(encoder_fr_sem, RT_WAITING_FOREVER);
        result = rt_sem_take(encoder_rl_sem, RT_WAITING_FOREVER);
        result = rt_sem_take(encoder_rr_sem, RT_WAITING_FOREVER);

        Dis2.X += 1.414 / 2 * (v_fl + v_fr + v_rl + v_rr) * dt2;
        Dis2.Y += 1.414 / 2 * (-v_fl + v_fr + v_rl - v_rr) * dt2;
        theta += omega_z * dt2;
        if (result == RT_EOK) // 获取编码器信号量成功
        {
            // x/1024*45/104*19.163*20
            // * -------------------------------- 更新当前速度 -------------------------------- *//
            v_fl = ((encoder_fl_val / encoder_line_count) * encoder_gear_count / wheel_gear_count) * wheel_week_length / (encoder_sample_time_ms / 1000);
            v_fr = -((encoder_fr_val / encoder_line_count) * encoder_gear_count / wheel_gear_count) * wheel_week_length / (encoder_sample_time_ms / 1000);
            v_rl = ((encoder_rl_val / encoder_line_count) * encoder_gear_count / wheel_gear_count) * wheel_week_length / (encoder_sample_time_ms / 1000);
            v_rr = -((encoder_rr_val / encoder_line_count) * encoder_gear_count / wheel_gear_count) * wheel_week_length / (encoder_sample_time_ms / 1000);
            omega_z = (wheel_r) / (4 * (L_ + l_)) * (-v_fl + v_fr + v_rr - v_rl);
            // * -------------------------------- 更新当前速度 -------------------------------- *//
            if (POSITION_PID_ENABLED)
            {
                // * -------------------------------- 根据位移计算xy方向的期望速度 -------------------------------- *//
                revised_x_speed = PIDController_weight_update(&x_axis_pid_controller, x_expect, Dis2.X, "position");
                revised_y_speed = PIDController_weight_update(&y_axis_pid_controller, y_expect, Dis2.Y, "position");
                // v_fl_expect = revised_x_speed - revised_y_speed;
                // v_fr_expect = revised_x_speed + revised_y_speed;
                // v_rl_expect = revised_x_speed + revised_y_speed;
                // v_rr_expect = revised_x_speed - revised_y_speed;
                // * -------------------------------- 根据位移计算xy方向的期望速度 -------------------------------- *//
            }
            if (ANGLE_PID_ENABLED)
            {
                omega_z_expected = PIDController_weight_update(&angle_pid_controller, theta_expected, theta, "theta");
            }

            if (ANGLE_SPEED_PID_ENABLED)
            {
                revised_omega_z = PIDController_weight_update(&angle_speed_pid_controller, omega_z_expected, omega_z, "omega");
                // 1,2,3,4
                // v_fl_expect -= ((L_ + l_) * revised_omega_z);   // fixed
                // v_fr_expect += ((L_ + l_) * revised_omega_z);   //
                // v_rr_expect += ((L_ + l_) * revised_omega_z);
                // v_rl_expect -= ((L_ + l_) * revised_omega_z);
            }
            v_fl_expect = revised_x_speed - revised_y_speed - ((L_ + l_) * revised_omega_z); // fixed
            v_fr_expect = revised_x_speed + revised_y_speed + ((L_ + l_) * revised_omega_z); //
            v_rr_expect = revised_x_speed - revised_y_speed + ((L_ + l_) * revised_omega_z);
            v_rl_expect = revised_x_speed + revised_y_speed - ((L_ + l_) * revised_omega_z);

            if (SPEED_PID_ENABLED)
            {
                revised_fl_duty = PIDController_weight_update(&fl_speed_pid_controller, v_fl_expect, v_fl, "speed");
                revised_fr_duty = PIDController_weight_update(&fr_speed_pid_controller, v_fr_expect, v_fr, "speed");
                revised_rl_duty = PIDController_weight_update(&rl_speed_pid_controller, v_rl_expect, v_rl, "speed");
                revised_rr_duty = PIDController_weight_update(&rr_speed_pid_controller, v_rr_expect, v_rr, "speed");

                // * -------------------------------- 更新转向 -------------------------------- *//
                // dir_fl = (revised_fl_duty > 0 ? 0 : 1);
                // dir_fr = (revised_fr_duty > 0 ? 0 : 1);
                // dir_rl = (revised_rl_duty > 0 ? 1 : 0);
                // dir_rr = (revised_rr_duty > 0 ? 0 : 1);

                dir_fl = (revised_fl_duty > 0 ? 0 : 1);
                dir_fr = (revised_fr_duty > 0 ? 1 : 0); // fixed
                dir_rl = (revised_rl_duty > 0 ? 1 : 0);
                dir_rr = (revised_rr_duty > 0 ? 0 : 1); // fixed

                gpio_set(FL_DIR, dir_fl);
                gpio_set(FR_DIR, dir_fr);
                gpio_set(RL_DIR, dir_rl);
                gpio_set(RR_DIR, dir_rr);
                // * -------------------------------- 更新转向 -------------------------------- *//

                // * -------------------------------- 更新转速 -------------------------------- *//
                pwm_duty(FL_PWM, fabs(revised_fl_duty));
                pwm_duty(FR_PWM, fabs(revised_fr_duty));
                pwm_duty(RL_PWM, fabs(revised_rl_duty));
                pwm_duty(RR_PWM, fabs(revised_rr_duty));
                // * -------------------------------- 更新转速 -------------------------------- *//
            }
            else
            {
                // pwm_duty(FL_PWM, 0);
                // pwm_duty(FR_PWM, 0);
                // pwm_duty(RL_PWM, 0);
                // pwm_duty(RR_PWM, 0);
            }

            if (PRINT_SPEED_ENABLED)
                // rt_kprintf("%ld,%ld,%ld,%ld,%ld\n", (int32)(imu_acc_x * 100), (int32)(V_value.X * 100), (int32)(Dis.X * 100), (int32)(flag * 100), (int32)(100 * v_fr));
                rt_kprintf("%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\n", (int32)(Dis2.X * 100), (int32)(Dis2.Y * 100), (int32)(100 * theta), (int32)(100 * v_fl), (int32)(100 * v_fr), (int32)(100 * v_rl), (int32)(100 * v_rr), (int32)(100 * omega_z), (int32)(100 * theta));
            // 因为不能打印浮点数,所以把它乘100
            // TODO open this
            // if (PRINT_SPEED_ENABLED)
            //     rt_kprintf("%ld,%ld,%ld,%ld\n", (int32)(100 * v_fl), (int32)(100 * v_fr), (int32)(100 * v_rl), (int32)(100 * v_rr));
        }
    }
}

void PIDController_Init(PIDController *pid, char *wheel_axis, char *type)
{
    if (pid == NULL)
    {
        rt_kprintf("Invalid pid controller!\n");
        return;
    }
    pid->integrator = 0.0f;
    pid->prevError = 0.0f;

    pid->differentiator = 0.0f;
    pid->prevMeasurement = 0.0f;

    pid->out = 0.0f;
    pid->prev_out = 0.0f;
    if (strcmp(type, "speed") == 0)
    {
        pid->limMax = PWM_DUTY_MAX;
        pid->limMin = -PWM_DUTY_MAX;
        pid->limMaxInt = PWM_DUTY_MAX;
        pid->limMinInt = -PWM_DUTY_MAX;
        pid->T = encoder_sample_time_ms / 1000;

        if (strcmp(wheel_axis, "fl") == 0)
        {
            pid->Kp = 36.37;
            pid->Ki = 1276.65;
            pid->Kd = 0;
            pid->tau = 100;
        }
        else if (strcmp(wheel_axis, "fr") == 0)
        {
            pid->Kp = 34.45;
            pid->Ki = 1264.97;
            pid->Kd = 0;
            pid->tau = 100;
        }
        else if (strcmp(wheel_axis, "rl") == 0)
        {
            pid->Kp = 38.99;
            pid->Ki = 1288.97;
            pid->Kd = 0;
            pid->tau = 100;
        }
        else if (strcmp(wheel_axis, "rr") == 0)
        {
            pid->Kp = 35.67;
            pid->Ki = 1316.21;
            pid->Kd = 0;
            pid->tau = 100;
        }
        else
        {
            pid->Kp = 0;
            pid->Ki = 0;
            pid->Kd = 0;
            pid->tau = 100;
            rt_kprintf("Undefined wheel or axis!\n");
        }
    }
    if (strcmp(type, "position") == 0)
    {

        pid->limMax = MAX_SPEED;
        pid->limMin = -MAX_SPEED;
        pid->limMaxInt = MAX_SPEED;
        pid->limMinInt = -MAX_SPEED;

        // OLD
        // pid->T = encoder_sample_time_ms / 1000;
        // pid->Kp = 0.9961;
        // pid->Ki = 0.6672;
        // pid->Kd = 0.1176;
        // pid->tau = 457.1312;

        pid->T = encoder_sample_time_ms / 1000;
        // pid->Kp = 0.99;
        // pid->Ki = 0.71;
        // pid->Kd = 0.02;
        // pid->tau = 457.13;

        pid->Kp = 0.49;
        pid->Ki = 0.18;
        pid->Kd = -0.02;
        pid->tau = 228.57;
    }

    if (strcmp(type, "omega") == 0)
    {
        pid->limMax = MAX_SPEED;
        pid->limMin = -MAX_SPEED;
        pid->limMaxInt = MAX_SPEED;
        pid->limMinInt = -MAX_SPEED;
        pid->T = encoder_sample_time_ms / 1000;

        // OLD
        // pid->T = encoder_sample_time_ms / 1000;
        pid->Kp = 0;
        pid->Ki = 0.82;
        pid->Kd = 0;
        pid->tau = 100;

        // pid->Kp = 0.23;
        // pid->Ki = 8.43;
        // pid->Kd = 0;
        // pid->tau = 100;
    }

    if (strcmp(type, "theta") == 0)
    {
        pid->limMax = MAX_SPEED;
        pid->limMin = -MAX_SPEED;
        pid->limMaxInt = MAX_SPEED;
        pid->limMinInt = -MAX_SPEED;
        pid->T = encoder_sample_time_ms / 1000;

        pid->Kp = 0.20;
        pid->Ki = 0.01;
        pid->Kd = -0.14;
        pid->tau = 0.34;
    }
}

float PIDController_weight_update(PIDController *pid, float setpoint, float measurement, char *type)
{
    float error = setpoint - measurement; // 误差
    if (strcmp("speed", type) == 0)
    {
        if (fabs(error) < 0.1)
            return pid->prev_out;
        if (setpoint >= MAX_SPEED)
            setpoint = MAX_SPEED; // 防止超速
    }

    else if (strcmp("position", type) == 0)
    {
        if (fabs(error) < 0.1) // 之前是3
            return pid->prev_out;
    }

    else if (strcmp("omega", type) == 0)
    {
        if (fabs(error) < 0.1) // 之前是3
            return pid->prev_out;
    }

    else if (strcmp("theta", type) == 0)
    {
        if (fabs(error) < 0.1) // 之前是3
            return pid->prev_out;
    }

    float proportional = pid->Kp * error;                                                   // 比例项
    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError); // 积分项

    // 积分限幅
    if (pid->integrator > pid->limMaxInt)
    {
        pid->integrator = pid->limMaxInt;
    }
    else if (pid->integrator < pid->limMinInt)
    {
        pid->integrator = pid->limMinInt;
    }

    pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement) /* Note: derivative on measurement, therefore minus sign in front of equation! */
                            + (2.0f * pid->tau - pid->T) * pid->differentiator) /
                          (2.0f * pid->tau + pid->T); // ? Derivative (band-limited differentiator)

    pid->out = proportional + pid->integrator + pid->differentiator; // 计算输出

    if (pid->out > pid->limMax)
    {
        pid->out = pid->limMax;
    }
    else if (pid->out < pid->limMin)
    {
        pid->out = pid->limMin;
    }

    /* Store error and measurement for later use */
    pid->prevError = error;
    pid->prevMeasurement = measurement;
    pid->prev_out = pid->out;
    /* Return controller output */
    return pid->out;
}

static void speed_set(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Invalid arguments!\n");
        return;
    }
    else if (!rt_strcmp(argv[1], "fl"))
    {
        v_fl_expect = atoi(argv[2]);
        rt_kprintf("v_fl_expect=%d\n", v_fl_expect);
        rt_enter_critical();
        rt_kprintf("99999,99999,99999,99999\n");
        rt_exit_critical();
        return;
    }
    else if (!rt_strcmp(argv[1], "fr"))
    {
        v_fr_expect = atoi(argv[2]);
        rt_kprintf("v_fr_expect=%d\n", v_fr_expect);
        rt_kprintf("99999,99999,99999,99999\n");
        return;
    }
    else if (!rt_strcmp(argv[1], "rl"))
    {
        v_rl_expect = atoi(argv[2]);
        rt_kprintf("v_rl_expect=%d\n", v_rl_expect);
        rt_kprintf("99999,99999,99999,99999\n");
        return;
    }
    else if (!rt_strcmp(argv[1], "rr"))
    {
        v_rr_expect = atoi(argv[2]);
        rt_kprintf("v_rr_expect=%d\n", v_rr_expect);
        rt_enter_critical();
        rt_kprintf("99999,99999,99999,99999\n");
        rt_exit_critical();
        return;
    }
    else
        return;
}

static void dir_set(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Invalid arguments!\n");
        return;
    }
    else if (!rt_strcmp(argv[1], "fl"))
    {
        gpio_set(FL_DIR, atoi(argv[2]));
    }
    else if (!rt_strcmp(argv[1], "fr"))
    {
        gpio_set(FR_DIR, atoi(argv[2]));
    }
    else if (!rt_strcmp(argv[1], "rl"))
    {
        gpio_set(RL_DIR, atoi(argv[2]));
    }
    else if (!rt_strcmp(argv[1], "rr"))
    {
        gpio_set(RR_DIR, atoi(argv[2]));
    }
    else if (!rt_strcmp(argv[1], "ma"))
    {
        gpio_set(Magnet, atoi(argv[2]));
    }
    else
        return;
    rt_kprintf("Succeeded!\n");
    return;
}

void minimum_encoder_printer_entry(void *parameter)
{
    // !
    uint8 result = 1;
    while (1)
    {
        result = rt_sem_take(encoder_fl_sem, RT_WAITING_FOREVER);
        result = rt_sem_take(encoder_fr_sem, RT_WAITING_FOREVER);
        result = rt_sem_take(encoder_rl_sem, RT_WAITING_FOREVER);
        result = rt_sem_take(encoder_rr_sem, RT_WAITING_FOREVER);

        if (result == RT_EOK) // 获取编码器信号量成功
        {
            rt_kprintf("%d,%d,%d,%d\n", encoder_fl_val, encoder_fr_val, encoder_rl_val, encoder_rr_val);
        }
    }
}

static void direction_control_m(int argc, char **argv) // m 是 manually 的意思
{
    if (argc < 2)
    {
        rt_kprintf("Invalid arguments!\n");
        return;
    }
    direction_control(*argv[1], atoi(argv[2]));
    PRINTF("Dir: %c, speed=%d\n", *argv[1], atoi(argv[2]));
    return;
}

void direction_control(char direction, float speed)
{
    // ! 未完成
    uint8 dir = 0;
    int8 fl, fr, rl, rr = 0;
    if (direction == 'w')
        dir = 1;
    // else if (direction == "s")
    //     dir = 2;
    else if (direction == 'a')
        dir = 3;
    // else if (direction == "d")
    //     dir = 4;
    // 前后左右
    switch (dir)
    {
    case 1:
        fl = rl = -1;
        fr = rr = 1;
        break;
    case 3:
        rl = -1;
        fr = 1;
        fl = 1;
        rr = -1;
        break;
    default:
        break;
    }
    v_fl_expect = speed * fl;
    v_fr_expect = speed * fr;
    v_rl_expect = speed * rl;
    v_rr_expect = speed * rr;
    return;
}

static void duty_set(int argc, char **argv)
{
    char sign[] = "999,999,999,999\n";
    SPEED_PID_ENABLED = 0;
    POSITION_PID_ENABLED = 0;
    if (argc < 2)
    {
        rt_kprintf("Invalid arguments!\n");
        return;
    }
    else if (!rt_strcmp(argv[1], "fl"))
    {
        rt_enter_critical();
        rt_kprintf("%s", sign);
        rt_exit_critical();

        pwm_duty(FL_PWM, duty_convert(atoi(argv[2])));
    }
    else if (!rt_strcmp(argv[1], "fr"))
    {
        rt_enter_critical();
        rt_kprintf("%s", sign);
        rt_exit_critical();

        pwm_duty(FR_PWM, duty_convert(atoi(argv[2])));
    }
    else if (!rt_strcmp(argv[1], "rl"))
    {
        rt_enter_critical();
        rt_kprintf("%s", sign);
        rt_exit_critical();

        pwm_duty(RL_PWM, duty_convert(atoi(argv[2])));
    }
    else if (!rt_strcmp(argv[1], "rr"))
    {
        rt_enter_critical();
        rt_kprintf("%s", sign);
        rt_exit_critical();

        pwm_duty(RR_PWM, duty_convert(atoi(argv[2])));
    }
    else if (!rt_strcmp(argv[1], "se"))
    {
        rt_enter_critical();
        rt_kprintf("%s", sign);
        rt_exit_critical();

        pwm_duty(Servo_PWM, duty_convert(atoi(argv[2])));
    }

    else
        return;
    rt_kprintf("Succeeded!\n");
    return;
}

static void enable_pid(int argc, char **argv)
{
    SPEED_PID_ENABLED = atoi(argv[1]);
    ANGLE_SPEED_PID_ENABLED = atoi(argv[2]);
    ANGLE_PID_ENABLED = atoi(argv[3]);
    POSITION_PID_ENABLED = atoi(argv[4]);
    rt_enter_critical();
    rt_kprintf("SPEED_PID_ENABLED=%d.\n", SPEED_PID_ENABLED);
    rt_kprintf("ANGLE_SPEED_PID_ENABLED=%d.\n", ANGLE_SPEED_PID_ENABLED);
    rt_kprintf("ANGLE_PID_ENABLED=%d.\n", ANGLE_PID_ENABLED);
    rt_kprintf("POSITION_PID_ENABLED=%d.\n", POSITION_PID_ENABLED);
    rt_exit_critical();
    return;
}

void read_GY_85_thread_entry(void *parameter)
{
    while (1)
    {
        Angle_get();
        // rt_kprintf("%ld\n", (int32)(Angle * 100));
        // rt_kprintf("%ld,%ld,%ld\n", (int32)(100 * imu_acc_x), (int32)(100 * imu_acc_y), (int32)(100 * imu_acc_z));
        rt_thread_mdelay(2);
    }
}

void readV__D(void *parameter)
{
    while (1)
    {
        // V_get();
        // Dis_get();
        // if (PRINT_SPEED_ENABLED)
        //     // rt_kprintf("%ld,%ld,%ld,%ld,%ld\n", (int32)(imu_acc_x * 100), (int32)(V_value.X * 100), (int32)(Dis.X * 100), (int32)(flag * 100), (int32)(100 * v_fr));
        //     rt_kprintf("%ld,%ld,%ld,%ld,%ld,%ld,%ld\n", (int32)(Dis2.X * 100), (int32)(Dis2.Y * 100), (int32)(100 * v_fl), (int32)(100 * v_fr), (int32)(100 * v_rl), (int32)(100 * v_rr), (int32)(100 * revised_x_speed));
        rt_thread_mdelay(25);
    }
}

static void readV__D_for_timer(void *parameter)
{
    V_get();
    Dis_get();
    // rt_kprintf("%ld,%ld,%ld,%ld\n", (int32)(imu_acc_x * 100), (int32)(V_value.X * 100), (int32)(Dis.X * 100), (int32)(flag * 100));
}

int create_main_timer(void)
{
    static rt_timer_t read_velocity_timer;
    read_velocity_timer = rt_timer_create("read_velocity_timer", readV__D_for_timer,
                                          RT_NULL, 20,
                                          RT_TIMER_FLAG_PERIODIC);
    // ! Dis_get 中的dt未更改
    if (read_velocity_timer != RT_NULL)
    {
        rt_timer_start(read_velocity_timer);
        rt_kprintf("read_velocity_timer start successful!\n");
    }
    return 0;
}

void get_true_coordinates(float new_height, float new_width, float *corner_x, float *corner_y, float *raw_coordinates_x, float *raw_coordinates_y, arm_matrix_instance_f32 *result)
{
    rt_enter_critical();
    // ? MATLAB 公式中的 corner_x 存的是 y 的坐标, 这里也这样操作吧
    float *p = corner_x;
    corner_x = corner_y;
    corner_y = p;

    float Y[] = {1, 1, new_width, new_width};
    float X[] = {1, new_height, new_height, 1};
    // 输入按顺时针方向排列的角点坐标
    // * -------------------------------- 求解变换矩阵 -------------------------------- *//
    arm_matrix_instance_f32 A;
    // ! 这个矩阵好像输错了！
    // float32_t A_data[] = {corner_x[0], corner_y[0], 1, 0, 0, 0, -X[0] * corner_x[0], -X[0] * corner_y[1],
    //                       0, 0, 0, corner_x[0], corner_y[0], 1, -Y[0] * corner_x[0], -Y[0] * corner_y[1],
    //                       corner_x[1], corner_y[1], 1, 0, 0, 0, -X[1] * corner_x[1], -X[1] * corner_y[1],
    //                       0, 0, 0, corner_x[1], corner_y[1], 1, -Y[1] * corner_x[1], -Y[1] * corner_y[1],
    //                       corner_x[2], corner_y[2], 1, 0, 0, 0, -X[2] * corner_x[2], -X[2] * corner_y[2],
    //                       0, 0, 0, corner_x[2], corner_y[2], 1, -Y[2] * corner_x[2], -Y[2] * corner_y[2],
    //                       corner_x[3], corner_y[3], 1, 0, 0, 0, -X[3] * corner_x[3], -X[3] * corner_y[3],
    //                       0, 0, 0, corner_x[3], corner_y[3], 1, -Y[3] * corner_x[3], -Y[3] * corner_y[3]};

    // * 这个是新打的
    float32_t A_data[] = {corner_x[0], corner_y[0], 1, 0, 0, 0, -X[0] * corner_x[0], -X[0] * corner_y[0],
                          0, 0, 0, corner_x[0], corner_y[0], 1, -Y[0] * corner_x[0], -Y[0] * corner_y[0],
                          corner_x[1], corner_y[1], 1, 0, 0, 0, -X[1] * corner_x[1], -X[1] * corner_y[1],
                          0, 0, 0, corner_x[1], corner_y[1], 1, -Y[1] * corner_x[1], -Y[1] * corner_y[1],
                          corner_x[2], corner_y[2], 1, 0, 0, 0, -X[2] * corner_x[2], -X[2] * corner_y[2],
                          0, 0, 0, corner_x[2], corner_y[2], 1, -Y[2] * corner_x[2], -Y[2] * corner_y[2],
                          corner_x[3], corner_y[3], 1, 0, 0, 0, -X[3] * corner_x[3], -X[3] * corner_y[3],
                          0, 0, 0, corner_x[3], corner_y[3], 1, -Y[3] * corner_x[3], -Y[3] * corner_y[3]};

    rt_kprintf("A_data[i]\n");
    for (int i = 0; i < 64; i++)
        rt_kprintf("%ld\n", (int32)(100 * A_data[i]));
    rt_kprintf("-----------\n");

    A.numCols = 8;
    A.numRows = 8;
    A.pData = A_data;

    rt_kprintf("A.pData[i]\n");
    for (int i = 0; i < 64; i++)
        rt_kprintf("%ld\n", (int32)(100 * A.pData[i]));
    rt_kprintf("-----------\n");

    // float32_t A_data[8][8] = {{corner_x[1], corner_y[1], 1, 0, 0, 0, -X[1] * corner_x[1], -X[1] * corner_y[1]},
    //                           {0, 0, 0, corner_x[1], corner_y[1], 1, -Y[1] * corner_x[1], -Y[1] * corner_y[1]},
    //                           {corner_x[2], corner_y[2], 1, 0, 0, 0, -X[2] * corner_x[2], -X[2] * corner_y[2]},
    //                           {0, 0, 0, corner_x[2], corner_y[2], 1, -Y[2] * corner_x[2], -Y[2] * corner_y[2]},
    //                           {corner_x[3], corner_y[3], 1, 0, 0, 0, -X[3] * corner_x[3], -X[3] * corner_y[3]},
    //                           {0, 0, 0, corner_x[3], corner_y[3], 1, -Y[3] * corner_x[3], -Y[3] * corner_y[3]},
    //                           {corner_x[4], corner_y[4], 1, 0, 0, 0, -X[4] * corner_x[4], -X[4] * corner_y[4]},
    //                           {0, 0, 0, corner_x[4], corner_y[4], 1, -Y[4] * corner_x[4], -Y[4] * corner_y[4]}};

    // arm_mat_init_f32(&A, 8, 8, A_data);
    // A.pData = A_data;
    // A.numRows = 8;
    // A.numCols = 8;
    arm_matrix_instance_f32 b;

    float32_t b_data[] = {X[1], Y[1], X[2], Y[2], X[3], Y[3], X[4], Y[4]};
    arm_mat_init_f32(&b, 8, 1, b_data);

    arm_matrix_instance_f32 A_inv;
    float32_t A_inv_data[8 * 8] = {0};
    arm_mat_init_f32(&A_inv, 8, 8, A_inv_data);
    arm_mat_inverse_f32(&A, &A_inv);
    rt_kprintf("A_inv_data.pData[i]\n");
    for (int i = 0; i < 64; i++)
        rt_kprintf("%ld\n", (int32)(100 * A_inv.pData[i]));
    rt_kprintf("-----------\n");

    // arm_matrix_instance_f32 transform_matrix;
    // float32_t transform_matrix_data[8] = {0};
    // arm_mat_init_f32(&transform_matrix, 8, 1, transform_matrix_data);

    int status = arm_mat_mult_f32(&A_inv, &b, result);
    rt_kprintf("%d\n", status);
    // * -------------------------------- 求解变换矩阵 -------------------------------- *//

    rt_exit_critical();
}

static void test_get_true_coordinates(int argc, char **argv)
{
    // float max_x, max_y, min_x, min_y;
    // uint32_t __ = 0; // 存放不需要的返回值
    // float corner_x[] = {61.1259314456036, 50.0380029806259, 188.100596125186, 192.035022354695};
    // float corner_y[] = {59.2779433681073, 254.210879284650, 258.860655737705, 66.7891207153502};

    // arm_max_f32(corner_x, sizeof(corner_x) / sizeof(corner_x[0]), &max_x, &__);
    // arm_max_f32(corner_y, sizeof(corner_y) / sizeof(corner_y[0]), &max_y, &__);
    // arm_min_f32(corner_x, sizeof(corner_x) / sizeof(corner_x[0]), &min_x, &__);
    // arm_min_f32(corner_y, sizeof(corner_y) / sizeof(corner_y[0]), &min_y, &__);

    // float new_width = max_x - min_x;
    // float new_height = max_y - min_y;

    // arm_matrix_instance_f32 transform_matrix;
    // float transform_matrix_data[9] = {0};
    // arm_mat_init_f32(&transform_matrix, 8, 1, transform_matrix_data);
    // get_true_coordinates(new_height, new_width, corner_x, corner_y, NULL, NULL, &transform_matrix);
    // transform_matrix_data[8] = 1;

    // for (int i = 0; i < 9; i++)
    //     rt_kprintf("%ld\n", (int32)(100 * transform_matrix.pData[i]));
    float corner[] = {59.2779433681073,
                      61.1259314456036,
                      254.210879284650,
                      50.0380029806259,
                      258.860655737705,
                      188.100596125186,
                      66.7891207153502,
                      192.035022354695};
    float result[9] = {0};
    get_transform_matrix(corner, result);

    for (int i = 0; i < 9; i++)
        rt_kprintf("%ld\n", (int32)(100 * result[i]));
}

static void test_matrix_function()
{
    rt_enter_critical();
    arm_matrix_instance_f32 A;
    float32_t A_data[] = {8, 1, 6, 3, 5, 7, 4, 9, 2};
    arm_mat_init_f32(&A, 3, 3, A_data);
    rt_kprintf("A.pData[i]\n");
    for (int i = 0; i < 9; i++)
        rt_kprintf("%ld\n", (int32)(100 * A.pData[i]));

    arm_matrix_instance_f32 A_inv;
    float32_t A_inv_data[9] = {0};
    arm_mat_init_f32(&A_inv, 3, 3, A_inv_data);

    arm_mat_inverse_f32(&A, &A_inv);

    rt_kprintf("A_inv.pData[i]\n");
    for (int i = 0; i < 9; i++)
        rt_kprintf("%ld\n", (int32)(100 * A_inv.pData[i]));
    rt_exit_critical();
}

static void position_set(int argc, char **argv)
{
    if (argc < 3)
    {
        rt_kprintf("Invalid arguments!\n");
        return;
    }
    x_expect = atoi(argv[1]);
    y_expect = atoi(argv[2]);
    rt_kprintf("x_expect=%ld\n", (int32)(x_expect));
    rt_kprintf("y_expect=%ld\n", (int32)(y_expect));
    return;
    // else if (!rt_strcmp(argv[1], "x"))
    // {
    //     x_expect = atoi(argv[2]);
    //     rt_kprintf("x_expect=%ld\n", (int32)(x_expect));
    //     // rt_enter_critical();
    //     // rt_kprintf("99999,99999,99999,99999\n");
    //     // rt_exit_critical();
    //     return;
    // }
    // else if (!rt_strcmp(argv[1], "y"))
    // {
    //     y_expect = atoi(argv[2]);
    //     rt_kprintf("y_expect=%ld\n", (int32)(y_expect));
    //     // rt_kprintf("99999,99999,99999,99999\n");
    //     return;
    // }
    // else return;
}

void receive_message_thread_entry(void *parameter)
{
    uint8 buff_front[64] = {0}; // uint8 aka unsigned char
    uint8 buff_down[64] = {0};
    uint8 received_front_msg = 0;
    uint8 received_down = 0;
    uint8 front_msg_cursor = 0;
    uint8 down_msg_cursor = 0;

    rt_thread_mdelay(500); // 等待初始化

    while (1)
    {
        if (1)
        {
            received_front_msg = uart_query(ART_Front_UART, &(buff_front[front_msg_cursor]));
            if (received_front_msg)
            {
                front_msg_cursor++;
                rt_kprintf("%s\n", buff_front);
            }
        }
        uart_query(ART_Down_UART, buff_down);
        rt_thread_mdelay(100);
    }
}

int create_system_identification_threads_group(void)
{
    rt_thread_t tid;
    // * 编码器读取线程创建
    tid = rt_thread_create("read_encoder_thread",     // 线程名称
                           read_encoder_thread_entry, // 线程入口函数
                           RT_NULL,                   // 线程参数
                           1024,                      // 1024 个字节的栈空间
                           5,                         // 线程优先级为5，数值越小，优先级越高，0为最高优先级。
                                                      // 可以通过修改rt_config.h中的RT_THREAD_PRIORITY_MAX宏定义(默认值为8)来修改最大支持的优先级
                           5);                        // 时间片为5

    rt_kprintf("create dynamic thread: read_encoder_thread.\n");
    if (tid != RT_NULL) // 线程创建成功
    {
        rt_kprintf("thread: read_encoder_thread dynamic thread create OK.\n");
        rt_thread_startup(tid); // 运行该线程
    }
    else // 线程创建失败
    {
        rt_kprintf("thread: read_encoder_thread dynamic thread create ERROR.\n");
        return 1;
    }
    // * 编码器读取线程创建

    // * 编码器打印线程创建
    tid = rt_thread_create("minimum_encoder_printer",     // 线程名称
                           minimum_encoder_printer_entry, // 线程入口函数
                           RT_NULL,                       // 线程参数
                           1024,                          // 1024 个字节的栈空间
                           5,                             // 线程优先级为5，数值越小，优先级越高，0为最高优先级。
                                                          // 可以通过修改rt_config.h中的RT_THREAD_PRIORITY_MAX宏定义(默认值为8)来修改最大支持的优先级
                           5);                            // 时间片为5
    rt_kprintf("create dynamic thread: minimum_encoder_printer.\n");
    if (tid != RT_NULL) // 线程创建成功
    {
        rt_kprintf("thread: minimum_encoder_printer thread create OK.\n");
        rt_thread_startup(tid); // 运行该线程
    }
    else // 线程创建失败
    {
        rt_kprintf("thread: minimum_encoder_printer thread create ERROR.\n");
        return 1;
    }
    // * 编码器打印线程创建

    return 0;
}

static void test_move(int argc, char **argv)
{
    gpio_set(LED10, 1);
    // 复位
    pwm_duty(Servo_PWM, duty_convert(40));
    rt_thread_mdelay(100);
    pwm_duty(Servo_PWM, duty_convert(20));
    rt_thread_mdelay(100);
    pwm_duty(Servo_PWM, duty_convert(5));
    rt_thread_mdelay(100);
    // 复位
    pwm_duty(Servo_PWM, duty_convert(5));
    gpio_set(Magnet, 1);
    rt_thread_mdelay(3000);
    pwm_duty(Servo_PWM, duty_convert(20));
    rt_thread_mdelay(100);
    pwm_duty(Servo_PWM, duty_convert(40));
    rt_thread_mdelay(3000);
    pwm_duty(Servo_PWM, duty_convert(20));
    rt_thread_mdelay(100);

    pwm_duty(Servo_PWM, duty_convert(5));
    rt_thread_mdelay(100);
    pwm_duty(Servo_PWM, duty_convert(20));
    rt_thread_mdelay(100);
    gpio_set(Magnet, 0);
    pwm_duty(Servo_PWM, duty_convert(40));
    gpio_set(LED10, 0);
}

static void omega_z_set(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Invalid arguments!\n");
        return;
    }
    omega_z_expected = atoi(argv[1]);
    rt_kprintf("omega_z_expected=%d\n", (int32)omega_z_expected);
    return;
}