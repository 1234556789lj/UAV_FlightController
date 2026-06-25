#ifndef COM_PID_H
#define COM_PID_H
#define PID_PERIOD 0.006
typedef struct
{
    float kp;        // 比例系数，值越大响应越快
    float ki;        // 积分系数，解决稳态误差
    float kd;        // 微分系数，值越大抑制效果越好，解决过调震荡
    float err;       // 误差值
    float desire;    // 目标值
    float measure;   // 测量值
    float last_err;  // 上一次误差值
    float intergral; // 积分累加
    float output;    // 输出值
} PID_st;
/**
 * @brief 限制函数
 *
 */
float Com_limit(float value, float max, float min);

// 单词pid计算
void Com_pid_calc(PID_st *pid);

// 串级pid计算
void Com_pid_calc_chain(PID_st *out_pid, PID_st *in_pid);

#endif
