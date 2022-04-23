#include "headfile.h"

// 一些已经不需要的函数

uint32 i = 0; // ? 调试系统辨识时展示当前占空比的变量

// MSH_CMD_EXPORT(sweep_pwm_duty, "sweep_pwm_duty sample - sweep_pwm_duty fl");

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