#ifndef COM_PID_H
#define COM_PID_H

#define PID_PERIOD  0.006f

typedef struct
{
    float kp;        // 比例系数，值越大响应越快
    float ki;        // 积分系数，解决稳态误差
    float kd;        // 微分系数，值越大抑制效果越好，解决过调震荡
    float err;       // 误差值（目标值 - 测量值）
    float desire;    // 目标值
    float measure;   // 测量值
    float last_err;  // 上一次误差值
    float intergral; // 积分累加
    float output;    // 输出值
} PID_st;

/**
 * @brief 限制函数
 */
float Com_limit(float value, float max, float min);

/**
 * @brief 单环PID计算
 * @note 误差方向：desire - measure（目标值 - 测量值）
 *       输出 = kp*err + ki*integral*dt + kd*der/dt
 */
void Com_pid_calc(PID_st *pid);

/**
 * @brief 串级PID计算
 * @note 外环输出 → 内环目标值，先外环后内环
 */
void Com_pid_calc_chain(PID_st *out_pid, PID_st *in_pid);

/**
 * @brief 复位PID状态（清零积分和上一次误差）
 * @note 在飞行状态切换（解锁、加锁、故障恢复）时调用，防止积分饱合
 */
void Com_pid_reset(PID_st *pid);

#endif
