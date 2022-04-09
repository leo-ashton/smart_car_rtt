#include "headfile.h"

// **************************** 宏定义 ****************************
#define FL (PWM1_MODULE3_CHB_D1)
#define FR (PWM1_MODULE3_CHA_D0)
#define RL (PWM1_MODULE1_CHA_D14)
#define RR (PWM1_MODULE1_CHB_D15)
#define duty_convert(x) (x / 100.0 * PWM_DUTY_MAX)
#define encoder_gear_count (45.0)
#define encoder_line_count (1024.0)
#define wheel_week_length (19.163) // 轮子周长, 单位为厘米
#define wheel_gear_count (104.0)

// **************************** 宏定义 ****************************

// **************************** 结构体类型定义 ****************************

// **************************** 结构体类型定义 ****************************

// **************************** 变量定义 ****************************
static rt_sem_t encoder3 = RT_NULL; // 创建指向信号量的指针
// static rt_sem_t v_fl, v_fr, v_rl, v_rr = RT_NULL; // ! 信号量的 value 字段是 uint16, 不可以用来存放浮点数.

// **************************** 变量定义 ****************************

// **************************** 函数定义 ****************************
void read_encoder_thread_entry(void *parameter);
void print_encoder_value_entry(void *parameter);
static void motor_control(int argc, char **argv);
// **************************** 函数定义 ****************************

int main(void)
{
    //此处编写用户代码(例如：外设初始化代码等)

    // * --------------------------------外设初始化--------------------------------

    // * --------------------------------外设初始化--------------------------------

    // * --------------------------------GPIO 初始化--------------------------------
    gpio_init(B9, GPO, 0, GPIO_PIN_CONFIG);
    gpio_init(D12, GPO, 0, GPIO_PIN_CONFIG);
    gpio_init(D13, GPO, 0, GPIO_PIN_CONFIG);
    gpio_init(D2, GPO, 0, GPIO_PIN_CONFIG);

    // * --------------------------------GPIO 初始化--------------------------------

    // * --------------------------------编码器初始化--------------------------------
    //一个QTIMER可以 创建两个正交解码
    qtimer_quad_init(QTIMER_1, QTIMER1_TIMER0_C0, QTIMER1_TIMER1_C1);
    qtimer_quad_init(QTIMER_1, QTIMER1_TIMER2_C2, QTIMER1_TIMER3_C24);
    qtimer_quad_init(QTIMER_2, QTIMER2_TIMER0_C3, QTIMER2_TIMER3_C25);
    qtimer_quad_init(QTIMER_3, QTIMER3_TIMER2_B18, QTIMER3_TIMER3_B19);
    rt_kprintf("QTIMER initialization succeeded.\n");
    // * --------------------------------编码器初始化--------------------------------

    // * -------------------------------- 电机初始化 --------------------------------
    pwm_init(FL, 50, 0);
    pwm_init(FR, 50, 0);
    pwm_init(RL, 50, duty_convert(20));
    pwm_init(RR, 50, 0);
    // * -------------------------------- 电机初始化 --------------------------------

    // * --------------------------------信号量初始化--------------------------------
    encoder3 = rt_sem_create("encoder3", 0, RT_IPC_FLAG_PRIO);
    // v_fl = rt_sem_create("v_fl", 0, RT_IPC_FLAG_PRIO);
    // v_fr = rt_sem_create("v_fr", 0, RT_IPC_FLAG_PRIO);
    // v_rl = rt_sem_create("v_rl", 0, RT_IPC_FLAG_PRIO);
    // v_rr = rt_sem_create("v_rr", 0, RT_IPC_FLAG_PRIO);
    // * --------------------------------信号量初始化--------------------------------

    // * --------------------------------线程初始化--------------------------------
    create_dynamic_thread();
    // * --------------------------------线程初始化--------------------------------

    EnableGlobalIRQ(0);
    // * 外设初始化

    while (1)
    {
        //此处编写需要循环执行的代码

        gpio_toggle(B9);
        rt_thread_mdelay(100);
    }
}

void read_encoder_thread_entry(void *parameter)
{
    float v_fl, v_fr, v_rl, v_rr;
    v_fl = v_fr = v_rl = v_rr = 0;

    while (1)
    {
        //读取编码器计数值
        // encoder1 = qtimer_quad_get(QTIMER_1, QTIMER1_TIMER0_C0);
        // encoder2 = qtimer_quad_get(QTIMER_1, QTIMER1_TIMER2_C2);
        encoder3->value = (rt_uint16_t)qtimer_quad_get(QTIMER_2, QTIMER2_TIMER0_C3);
        // encoder4 = qtimer_quad_get(QTIMER_3, QTIMER3_TIMER2_B18);

        // * 计算速度

        // * 计算速度
        v_rl = ((encoder3->value / encoder_line_count) * encoder_gear_count / wheel_gear_count) * wheel_week_length * 10;

        rt_sem_release(encoder3);
        // rt_sem_release(v_rl);

        qtimer_quad_clear(QTIMER_1, QTIMER1_TIMER0_C0);
        qtimer_quad_clear(QTIMER_1, QTIMER1_TIMER2_C2);
        qtimer_quad_clear(QTIMER_2, QTIMER2_TIMER0_C3);
        qtimer_quad_clear(QTIMER_3, QTIMER3_TIMER2_B18);
        rt_thread_mdelay(100);
    }
}

int create_dynamic_thread(void)
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

    tid = rt_thread_create("print_encoder_thread",    // 线程名称
                           print_encoder_value_entry, // 线程入口函数
                           RT_NULL,                   // 线程参数
                           1024,                      // 1024 个字节的栈空间
                           5,                         // 线程优先级为5，数值越小，优先级越高，0为最高优先级。
                                                      // 可以通过修改rt_config.h中的RT_THREAD_PRIORITY_MAX宏定义(默认值为8)来修改最大支持的优先级
                           5);                        // 时间片为5
    rt_kprintf("create dynamic thread: print_encoder_thread.\n");
    if (tid != RT_NULL) // 线程创建成功
    {
        rt_kprintf("thread: print_encoder_thread dynamic thread create OK.\n");
        rt_thread_startup(tid); // 运行该线程
    }
    else // 线程创建失败
    {
        rt_kprintf("thread1: print_encoder_thread dynamic thread create ERROR.\n");
        return 1;
    }

    return 0;
}

void print_encoder_value_entry(void *parameter)
{
    // 读取并打印编码器的值
    static rt_err_t result;
    float v_fl, v_fr, v_rl, v_rr;
    v_fl = v_fr = v_rl = v_rr = 0;

    while (1)
    {
        // 通过永久等待的方式获取信号量，获取到了则执行sum++的操作
        result = rt_sem_take(encoder3, RT_WAITING_FOREVER);
        uint16_t cnt = 0;
        if (result == RT_EOK) // 获取信号量成功
        {
            v_rl = (((int16)encoder3->value / encoder_line_count) * encoder_gear_count / wheel_gear_count) * wheel_week_length * 10;
            // * 将速度的计算放在这里, 因为信号量必须是整数, 浮点数不能通过它在线程间传递.
            // rt_kprintf("Encoder3= %d \n", (int16)encoder3->value);
            PRINTF("%f\n", v_rl);
            // cnt++;
            // if (cnt == 3)
            // {
            //     PRINTF("\n");
            //     cnt = 0;
            // }
            // else
            //     PRINTF(",");
        }
    }
}

static void motor_control(int argc, char **argv)
{
    // ! 用命令行控制电机, 未完成
    if (argc != 2 || argv == NULL)
    {
        rt_kprintf("Invalid arguments!\n");
        return;
    }
}