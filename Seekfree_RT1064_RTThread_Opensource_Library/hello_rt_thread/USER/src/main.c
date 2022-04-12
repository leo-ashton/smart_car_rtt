#include "headfile.h"
#include "math.h"
#include <rtthread.h>
// **************************** 宏定义 ****************************
#define FL_PWM (PWM1_MODULE3_CHB_D1)
#define FR_PWM (PWM1_MODULE3_CHA_D0)
#define RL_PWM (PWM1_MODULE1_CHA_D14)
#define RR_PWM (PWM1_MODULE1_CHB_D15)

#define FL_DIR (D3)
#define FR_DIR (D2)
#define RL_DIR (D12)
#define RR_DIR (D13)

#define duty_convert(x) (x / 100.0 * PWM_DUTY_MAX) // x为期望的占空比, 该函数将期望占空比直接转换为 pwm_duty 函数接受的参数

#define encoder_gear_count (45.0)
#define encoder_line_count (1024.0)
#define wheel_week_length (19.163) // 轮子周长, 单位为厘米
#define wheel_gear_count (104.0)
// **************************** 宏定义 ****************************

// **************************** 结构体类型定义 ****************************

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

} PIDController;

// **************************** 结构体类型定义 ****************************

// **************************** 变量定义 ****************************

static rt_sem_t encoder_fl = RT_NULL; // 创建指向信号量的指针
static rt_sem_t encoder_fr = RT_NULL; // 创建指向信号量的指针
static rt_sem_t encoder_rl = RT_NULL; // 创建指向信号量的指针
static rt_sem_t encoder_rr = RT_NULL; // 创建指向信号量的指针

static int32 v_fl_expect = 0, v_fr_expect = 0, v_rl_expect = 0, v_rr_expect = 0;

uint32 i = 0; // ? 调试系统辨识时展示当前占空比的变量
uint8 pid_enabled = 1;
// **************************** 变量定义 ****************************

// **************************** 函数定义 ****************************

void read_encoder_thread_entry(void *parameter);
void pid_motor_control_entry(void *parameter);

int create_main_dynamic_thread(void);
int create_system_identification_thread(void);

void PIDController_Init(PIDController *pid);
float PIDController_weight_update(PIDController *pid, float setpoint, float measurement);

static void speed_set(int argc, char **argv);
static void dir_set(int argc, char **argv);

void sweep_pwm_duty(int argc, char **argv);
void minimum_pulse_counter_entry(void *parameter);
static void duty_set(int argc, char **argv);
static void enable_pid(int argc, char **argv);

// **************************** 函数定义 ****************************

// **************************** Finsh 初始化 ****************************

MSH_CMD_EXPORT(speed_set, "speed_set sample - speed_set fl 100");
MSH_CMD_EXPORT(dir_set, "dir_set sample - dir_set fl 1");
MSH_CMD_EXPORT(sweep_pwm_duty, "sweep_pwm_duty sample - sweep_pwm_duty fl");
MSH_CMD_EXPORT(duty_set, "sweep_pwm_duty sample - duty_set fl 50");
MSH_CMD_EXPORT(enable_pid, "dir_set sample - dir_set fl 1");

// **************************** Finsh 初始化 ****************************

int main(void)
{
    //此处编写用户代码(例如：外设初始化代码等)

    // * --------------------------------外设初始化--------------------------------

    // * --------------------------------外设初始化--------------------------------

    // * --------------------------------GPIO 初始化--------------------------------
    gpio_init(B9, GPO, 0, GPIO_PIN_CONFIG);
    gpio_init(FL_DIR, GPO, 0, GPIO_PIN_CONFIG);
    gpio_init(FR_DIR, GPO, 0, GPIO_PIN_CONFIG);
    gpio_init(RL_DIR, GPO, 0, GPIO_PIN_CONFIG);
    gpio_init(RR_DIR, GPO, 0, GPIO_PIN_CONFIG);
    // * --------------------------------GPIO 初始化--------------------------------

    // * --------------------------------编码器初始化--------------------------------
    qtimer_quad_init(QTIMER_1, QTIMER1_TIMER0_C0, QTIMER1_TIMER1_C1);
    qtimer_quad_init(QTIMER_1, QTIMER1_TIMER2_C2, QTIMER1_TIMER3_C24);
    qtimer_quad_init(QTIMER_2, QTIMER2_TIMER0_C3, QTIMER2_TIMER3_C25);
    qtimer_quad_init(QTIMER_3, QTIMER3_TIMER2_B18, QTIMER3_TIMER3_B19);
    rt_kprintf("QTIMER initialization succeeded.\n");
    // * --------------------------------编码器初始化--------------------------------

    // * -------------------------------- 电机初始化 --------------------------------
    pwm_init(FL_PWM, 50, 0);
    pwm_init(FR_PWM, 50, 0);
    pwm_init(RL_PWM, 50, 0);
    pwm_init(RR_PWM, 50, 0);
    // * -------------------------------- 电机初始化 --------------------------------

    // * --------------------------------信号量初始化--------------------------------

    encoder_fl = rt_sem_create("encoder_fl", 0, RT_IPC_FLAG_PRIO);
    encoder_fr = rt_sem_create("encoder_fr", 0, RT_IPC_FLAG_PRIO);
    encoder_rl = rt_sem_create("encoder_rl", 0, RT_IPC_FLAG_PRIO);
    encoder_rr = rt_sem_create("encoder_rr", 0, RT_IPC_FLAG_PRIO);

    // * --------------------------------信号量初始化--------------------------------

    // * --------------------------------线程初始化--------------------------------

    // ! 启动需要的线程
    create_main_dynamic_thread();
    // create_system_identification_thread();

    // * --------------------------------线程初始化--------------------------------

    EnableGlobalIRQ(0);
    // * 外设初始化

    while (1)
    {
        //此处编写需要循环执行的代码
        gpio_toggle(B9);
        rt_thread_mdelay(500);
    }
}

void read_encoder_thread_entry(void *parameter)
{
    float v_fl, v_fr, v_rl, v_rr;
    v_fl = v_fr = v_rl = v_rr = 0;

    while (1)
    {
        //读取编码器计数值
        encoder_fl->value = (rt_uint16_t)qtimer_quad_get(QTIMER_1, QTIMER1_TIMER2_C2);
        encoder_fr->value = (rt_uint16_t)qtimer_quad_get(QTIMER_1, QTIMER1_TIMER0_C0);
        encoder_rl->value = (rt_uint16_t)qtimer_quad_get(QTIMER_2, QTIMER2_TIMER0_C3);
        encoder_rr->value = (rt_uint16_t)qtimer_quad_get(QTIMER_3, QTIMER3_TIMER2_B18);

        rt_sem_release(encoder_fl);
        rt_sem_release(encoder_fr);
        rt_sem_release(encoder_rl);
        rt_sem_release(encoder_rr);

        qtimer_quad_clear(QTIMER_1, QTIMER1_TIMER0_C0);
        qtimer_quad_clear(QTIMER_1, QTIMER1_TIMER2_C2);
        qtimer_quad_clear(QTIMER_2, QTIMER2_TIMER0_C3);
        qtimer_quad_clear(QTIMER_3, QTIMER3_TIMER2_B18);
        rt_thread_mdelay(10);
    }
}

int create_main_dynamic_thread(void)
{
    // * 线程初始化函数
    // 线程控制块指针
    rt_thread_t tid;
    // 创建动态线程
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
        rt_kprintf("thread1: read_encoder_thread dynamic thread create ERROR.\n");
        return 1;
    }

    tid = rt_thread_create("pid_motor_control_entry", // 线程名称
                           pid_motor_control_entry,   // 线程入口函数
                           RT_NULL,                   // 线程参数
                           1024,                      // 1024 个字节的栈空间
                           5,                         // 线程优先级为5，数值越小，优先级越高，0为最高优先级。
                                                      // 可以通过修改rt_config.h中的RT_THREAD_PRIORITY_MAX宏定义(默认值为8)来修改最大支持的优先级
                           5);                        // 时间片为5
    rt_kprintf("create dynamic thread: pid_motor_control_thread.\n");
    if (tid != RT_NULL) // 线程创建成功
    {
        rt_kprintf("thread: pid_motor_control_thread dynamic thread create OK.\n");
        rt_thread_startup(tid); // 运行该线程
    }
    else // 线程创建失败
    {
        rt_kprintf("thread1: pid_motor_control_thread dynamic thread create ERROR.\n");
        return 1;
    }

    return 0;
}

void pid_motor_control_entry(void *parameter)
{
    // 读取并打印编码器的值
    static rt_err_t result;
    float v_fl, v_fr, v_rl, v_rr = 0; // ! 均为测量值
    float revised_fl_duty, revised_fr_duty, revised_rl_duty, revised_rr_duty = 0;
    // revised_fl_duty = revised_fr_duty = revised_rl_duty = revised_rr_duty = 0;
    uint8 dir_fl, dir_fr, dir_rl, dir_rr = 0;
    int32 revised_rl_duty_int = 0;

    PIDController fl_pid_controller;
    PIDController fr_pid_controller;
    PIDController rl_pid_controller;
    PIDController rr_pid_controller;

    PIDController_Init(&fl_pid_controller);
    PIDController_Init(&fr_pid_controller);
    PIDController_Init(&rl_pid_controller);
    PIDController_Init(&rr_pid_controller);

    // v_rl_expect=-150;

    while (1)
    {
        result = rt_sem_take(encoder_rl, RT_WAITING_FOREVER);
        if (result == RT_EOK) // 获取编码器信号量成功
        {
            // * -------------------------------- 更新当前速度 -------------------------------- *//
            v_fl = (((int16)encoder_fl->value / encoder_line_count) * encoder_gear_count / wheel_gear_count) * wheel_week_length * 10;
            v_fr = (((int16)encoder_fr->value / encoder_line_count) * encoder_gear_count / wheel_gear_count) * wheel_week_length * 10;
            v_rl = (((int16)encoder_rl->value / encoder_line_count) * encoder_gear_count / wheel_gear_count) * wheel_week_length * 10;
            v_rr = (((int16)encoder_rr->value / encoder_line_count) * encoder_gear_count / wheel_gear_count) * wheel_week_length * 10;
            // v_rl = (((int16)(encoder_rl->value) / encoder_line_count) * encoder_gear_count / wheel_gear_count) * wheel_week_length * 10;
            // * -------------------------------- 更新当前速度 -------------------------------- *//

            // * 将速度的计算放在这里, 因为信号量必须是整数, 浮点数不能通过它在线程间传递.
            if (pid_enabled)
            {
                revised_fl_duty = PIDController_weight_update(&fl_pid_controller, v_fl_expect, v_fl);
                revised_fr_duty = PIDController_weight_update(&fr_pid_controller, v_fr_expect, v_fr);
                revised_rl_duty = PIDController_weight_update(&rl_pid_controller, v_rl_expect, v_rl);
                revised_rr_duty = PIDController_weight_update(&rr_pid_controller, v_rr_expect, v_rr);
            }

            revised_rl_duty_int = revised_rl_duty;

            // * -------------------------------- 更新转向 -------------------------------- *//

            dir_fl = (revised_fl_duty > 0 ? 0 : 1);
            dir_fr = (revised_fr_duty > 0 ? 0 : 1);
            dir_rl = (revised_rl_duty > 0 ? 0 : 1);
            dir_rr = (revised_rr_duty > 0 ? 0 : 1);

            gpio_set(FL_DIR, dir_fl);
            gpio_set(FR_DIR, dir_fr);
            gpio_set(RL_DIR, dir_rl);
            gpio_set(RR_DIR, dir_rr);

            // rt_kprintf("dir_fl=%d\n", dir_fl);
            // * -------------------------------- 更新转向 -------------------------------- *//

            // * -------------------------------- 更新转速 -------------------------------- *//

            uint32 abs_revised_rl_duty = fabs(revised_rl_duty);
            rt_kprintf("%ld,%ld,%ld,%ld\n", (int32)(100 * v_fl), (int32)(100 * v_fr), (int32)(100 * v_rl), (int32)(100 * v_rr));

            // rt_kprintf("%ld\n", (int32)revised_fl_duty);
            // rt_kprintf("%d,%ld,%ld,%d\n", (int16)encoder_rl->value, abs_revised_rl_duty, revised_rl_duty_int, (int32)v_rl);
            if (pid_enabled)
            {
                pwm_duty(FL_PWM, fabs(revised_fl_duty));
                pwm_duty(FR_PWM, fabs(revised_fr_duty));
                pwm_duty(RL_PWM, fabs(revised_rl_duty));
                pwm_duty(RR_PWM, fabs(revised_rr_duty));
            }

            // * -------------------------------- 更新转速 -------------------------------- *//
        }
    }
}

void PIDController_Init(PIDController *pid)
{
    /* Clear controller variables */
    pid->integrator = 0.0f;
    pid->prevError = 0.0f;

    pid->differentiator = 0.0f;
    pid->prevMeasurement = 0.0f;

    pid->out = 0.0f;

    pid->limMax = PWM_DUTY_MAX;
    pid->limMin = -PWM_DUTY_MAX;
    pid->limMaxInt = 0.5 * PWM_DUTY_MAX;
    pid->limMinInt = -0.5 * PWM_DUTY_MAX;

    pid->T = 0.1;
    pid->Kp = 73.03;
    pid->Ki = 7;
    pid->Kd = 0.05;
    pid->tau = 0.02;
}

float PIDController_weight_update(PIDController *pid, float setpoint, float measurement)
{

    float error = setpoint - measurement;                                                   // 误差
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
    else
        return;
    rt_kprintf("Succeeded!\n");
    return;
}

void sweep_pwm_duty(int argc, char **argv)
{
    if (argc < 1)
    {
        rt_kprintf("Invalid arguments!\n");
        return;
    }

    else if (!rt_strcmp(argv[1], "fl"))
    {
        for (i = 0; i < PWM_DUTY_MAX; i += 10)
        {
            pwm_duty(FL_PWM, i);
            // rt_kprintf("%d,", i);
            rt_thread_mdelay(10);
        }
        for (; i > 0; i -= 10)
        {
            pwm_duty(FL_PWM, i);
            // rt_kprintf("%d,", i);
            rt_thread_mdelay(10);
        }
    }
    else if (!rt_strcmp(argv[1], "fr"))
    {
        for (i = 0; i < PWM_DUTY_MAX; i += 10)
        {
            pwm_duty(FR_PWM, i);
            // rt_kprintf("%d,", i);

            rt_thread_mdelay(10);
        }
        for (; i > 0; i -= 10)
        {
            pwm_duty(FR_PWM, i);
            // rt_kprintf("%d,", i);

            rt_thread_mdelay(10);
        }
    }
    else if (!rt_strcmp(argv[1], "rl"))
    {
        for (i = 0; i < PWM_DUTY_MAX; i += 10)
        {
            pwm_duty(RL_PWM, i);
            // rt_kprintf("%d,", i);

            rt_thread_mdelay(10);
        }
        for (; i > 0; i -= 10)
        {
            pwm_duty(RL_PWM, i);
            // rt_kprintf("%d,", i);

            rt_thread_mdelay(10);
        }
    }
    else if (!rt_strcmp(argv[1], "rr"))
    {
        for (i = 0; i < PWM_DUTY_MAX; i += 10)
        {
            pwm_duty(RR_PWM, i);
            // rt_kprintf("%d,", i);

            rt_thread_mdelay(10);
        }
        for (; i > 0; i -= 10)
        {
            pwm_duty(RR_PWM, i);
            // rt_kprintf("%d,", i);

            rt_thread_mdelay(10);
        }
    }
    else
        return;
    i = 0;

    pwm_duty(FL_PWM, 0);
    pwm_duty(FR_PWM, 0);
    pwm_duty(RL_PWM, 0);
    pwm_duty(RR_PWM, 0);
    rt_kprintf("Succeeded!\n");
    return;
}

void minimum_pulse_counter_entry(void *parameter)
{
    uint8 result = 1;
    while (1)
    {
        result = rt_sem_take(encoder_rl, RT_WAITING_FOREVER);
        if (result == RT_EOK) // 获取编码器信号量成功
        {
            rt_kprintf("%d,%d,%d,%d,%d\n", i, (int16)encoder_fl->value, (int16)encoder_fr->value, (int16)encoder_rl->value, (int16)encoder_rr->value, (int16)encoder_fl->value);
        }
    }
}

int create_system_identification_thread(void)
{
    // * 线程初始化函数
    // 线程控制块指针
    rt_thread_t tid;
    // 创建动态线程
    tid = rt_thread_create("minimum_pulse_counter",     // 线程名称
                           minimum_pulse_counter_entry, // 线程入口函数
                           RT_NULL,                     // 线程参数
                           1024,                        // 1024 个字节的栈空间
                           5,                           // 线程优先级为5，数值越小，优先级越高，0为最高优先级。
                                                        // 可以通过修改rt_config.h中的RT_THREAD_PRIORITY_MAX宏定义(默认值为8)来修改最大支持的优先级
                           5);                          // 时间片为5

    rt_kprintf("create dynamic thread: minimum_pulse_counter.\n");
    if (tid != RT_NULL) // 线程创建成功
    {
        rt_kprintf("thread: read_encoder_thread dynamic thread create OK.\n");
        rt_thread_startup(tid); // 运行该线程
    }
    else // 线程创建失败
    {
        rt_kprintf("thread1: read_encoder_thread dynamic thread create ERROR.\n");
        return 1;
    }

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
        rt_kprintf("thread1: read_encoder_thread dynamic thread create ERROR.\n");
        return 1;
    }

    return 0;
}

static void duty_set(int argc, char **argv)
{

    if (argc < 2)
    {
        rt_kprintf("Invalid arguments!\n");
        return;
    }
    else if (!rt_strcmp(argv[1], "fl"))
    {
        rt_enter_critical();
        rt_kprintf("99999,99999,99999,99999\n");
        rt_exit_critical();
        pwm_duty(FL_PWM, duty_convert(atoi(argv[2])));
    }
    else if (!rt_strcmp(argv[1], "fr"))
    {
        rt_enter_critical();
        rt_kprintf("99999,99999,99999,99999\n");
        rt_exit_critical();
        pwm_duty(FR_PWM, duty_convert(atoi(argv[2])));
    }
    else if (!rt_strcmp(argv[1], "rl"))
    {
        rt_enter_critical();
        rt_kprintf("99999,99999,99999,99999\n");
        rt_exit_critical();

        pwm_duty(RL_PWM, duty_convert(atoi(argv[2])));
    }
    else if (!rt_strcmp(argv[1], "rr"))
    {
        rt_enter_critical();
        rt_kprintf("99999,99999,99999,99999\n");
        rt_exit_critical();

        pwm_duty(RR_PWM, duty_convert(atoi(argv[2])));
    }
    else
        return;
    rt_kprintf("Succeeded!\n");
    return;
}

static void enable_pid(int argc, char **argv)
{
    pid_enabled = atoi(argv[1]);
    rt_enter_critical();
    rt_kprintf("PID controller status: %d.\n", pid_enabled);
    rt_exit_critical();
    return;
}