#include "Com_pid.h"

/**
 * @brief 限制函数
 *
 */
float Com_limit(float value, float max, float min)
{
    if (value > max)
    {
        value = max;
    }
    else if (value < min)
    {
        value = min;
    }
    return value;
}

// 单词pid计算
void Com_pid_calc(PID_st *pid)
{
    // 1.目标和测量 计算误差值
    pid->err = pid->measure - pid->desire;
    // 2.计算积分误差
    pid->intergral += pid->err;
    if (pid->last_err == 0)
    {
        pid->last_err = pid->err;
    }
    // 3.计算微分误差
    float der = pid->err - pid->last_err;

    // 4.计算输出
    pid->output = pid->kp * pid->err + (pid->ki * pid->intergral * PID_PERIOD) + pid->kd * der / PID_PERIOD;
    // 5.保存上一次误差
    pid->last_err = pid->err;
}

// 串级pid计算
void Com_pid_calc_chain(PID_st *out_pid, PID_st *in_pid)
{
    // 1.先计算外环，角度
    Com_pid_calc(out_pid);
    // 2.将外环输出值作为内环目标值
    in_pid->desire = out_pid->output;
    // 3.计算内环，角速度
    Com_pid_calc(in_pid);
}
