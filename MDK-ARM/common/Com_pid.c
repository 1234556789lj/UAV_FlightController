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
    // 1.计算误差值（标准方向：目标值 - 测量值）
    pid->err = pid->desire - pid->measure;

    // 2.积分累加
    pid->intergral += pid->err;

    // 3.首次调用时初始化上一次误差，避免微分突变
    if (pid->last_err == 0)
    {
        pid->last_err = pid->err;
    }

    // 4.计算微分误差
    float der = pid->err - pid->last_err;

    // 5.计算输出
    pid->output = pid->kp * pid->err
                + pid->ki * pid->intergral * PID_PERIOD
                + pid->kd * der / PID_PERIOD;

    // 6.保存上一次误差
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

// 复位PID状态
void Com_pid_reset(PID_st *pid)
{
    pid->intergral = 0;
    pid->last_err = 0;
    pid->err = 0;
    pid->output = 0;
}
