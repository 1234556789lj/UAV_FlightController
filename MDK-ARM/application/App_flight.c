#include "App_flight.h"

// 遥控器的值
extern Remote_Data remote_data;
// 飞行状态
extern Flight_State flight_state;
// 按下定高的高度
extern uint16_t fix_height;
// 存储mpu6050的全部数据
Gyro_Acc_Data all_data = {0};
// 低通滤波上一次的值
Gyro_Data last_data = {0};
// 欧拉角
Euler_Data euler_angle = {0};
// 角速度累加
float gyro_z_sum = 0;
// PID的调参是先调节内环再调节外环
// 俯仰角PID结构体  => 后续需要进行专业的PID调参
PID_st pitch_pid = {.kp = -6.00, .ki = 0.00, .kd = 0.00};
// Y轴角速度结构体 => 对应俯仰角的内环
// 极性问题 => 参数的正负可以调节 => 作用于电机的时候 正负
PID_st gyro_y_pid = {.kp = 2.00, .ki = 0.00, .kd = 0.10};

// 横滚角PID结构体
PID_st roll_pid = {.kp = -6.00, .ki = 0.00, .kd = 0.00};
// X轴角速度结构体 => 对应横滚角的内环
PID_st gyro_x_pid = {.kp = 2.00, .ki = 0.00, .kd = 0.10};

// 偏航角PID结构体
PID_st yaw_pid = {.kp = -3.00, .ki = 0.00, .kd = 0.00};
// z轴角速度结构体 => 对应横滚角的内环
PID_st gyro_z_pid = {.kp = -5.00, .ki = 0.00, .kd = 0.00};

// 定高pid结构体
PID_st fix_height_pid = {.kp = -0.50, .ki = 0.00, .kd = -0.30};

// 电机结构体,创建无人机的四个电机，左右前后四个电机，使用结构体封装电机数据，包含定时器句柄、定时器通道和电机速度
Motor_St left_top_motor = {.tim = &htim3, .channel = TIM_CHANNEL_1, .speed = 0};
Motor_St left_bottom_motor = {.tim = &htim4, .channel = TIM_CHANNEL_4, .speed = 0};
Motor_St right_top_motor = {.tim = &htim2, .channel = TIM_CHANNEL_2, .speed = 0};
Motor_St right_bottom_motor = {.tim = &htim1, .channel = TIM_CHANNEL_3, .speed = 0};

/**
 * @brief 飞控任务初始化 MPU6050初始化    启动电机
 *
 */
void App_flight_init(void)
{
    // 初始化mpu6050
    Int_mpu6050_init();

    // 启动电机
    Int_Motor_Start(&left_top_motor);
    Int_Motor_Start(&left_bottom_motor);
    Int_Motor_Start(&right_top_motor);
    Int_Motor_Start(&right_bottom_motor);

    // 初始化激光测距
    Int_VL53L1X_Init();
}

//
/**
 * @brief 获取欧拉角
 *
 */
void App_flight_get_euler_angle(void)
{
    // 获取mpu6050的全部数据
    Int_mpu6050_GetAllData(&all_data);

    // 对角速度进行低通滤波
    all_data.gyro.gyro_x = Common_Filter_LowPass(all_data.gyro.gyro_x, last_data.gyro_x);
    all_data.gyro.gyro_y = Common_Filter_LowPass(all_data.gyro.gyro_y, last_data.gyro_y);
    all_data.gyro.gyro_z = Common_Filter_LowPass(all_data.gyro.gyro_z, last_data.gyro_z);
    // 更新上一次的值
    last_data.gyro_x = all_data.gyro.gyro_x;
    last_data.gyro_y = all_data.gyro.gyro_y;
    last_data.gyro_z = all_data.gyro.gyro_z;

    // 打印角速度到串口
    // debug_printf(":%d,%d,%d\n", all_data.gyro.gyro_x, all_data.gyro.gyro_y, all_data.gyro.gyro_z);

    // 加速度波动比较大，使用卡尔曼滤波
    all_data.acc.acc_x = Common_Filter_KalmanFilter(&kfs[0], all_data.acc.acc_x);
    all_data.acc.acc_y = Common_Filter_KalmanFilter(&kfs[1], all_data.acc.acc_y);
    all_data.acc.acc_z = Common_Filter_KalmanFilter(&kfs[2], all_data.acc.acc_z);
    // 打印加速度
    // debug_printf(":%d,%d,%d\n", all_data.acc.acc_x, all_data.acc.acc_y, all_data.acc.acc_z);

    // // 通过加速度和角速度来计算飞机姿态，姿态解算
    // // 使用互补解算计算欧拉角，优先使用加速度解算，俯仰角和横滚角可以计算
    // euler_angle.pitch = atan2(all_data.acc.acc_x * 1.0, all_data.acc.acc_z) / 3.1415926 * 180;
    // euler_angle.roll = atan2(all_data.acc.acc_y * 1.0, all_data.acc.acc_z) / 3.1415926 * 180;

    // // 偏航角，只能使用角速度积分
    // // 16ADC转换为度每秒，量程是正负2000度每秒
    // gyro_z_sum += (all_data.gyro.gyro_z * 2000 / 32768) * 0.006;
    // euler_angle.yaw = gyro_z_sum;

    // 使用四元数解算姿态
    Common_IMU_GetEulerAngle(&all_data, &euler_angle, 0.006);

    // 打印欧拉角
    // debug_printf(":%.2f,%.2f,%.2f\n", euler_angle.pitch, euler_angle.roll, euler_angle.yaw);
}

/**
 * @brief 根据欧拉角计算pid的目标值
 *
 */
void App_flight_pid_process(void)
{
    // 俯仰角
    // 1.需要目标值和测量值
    // 外环为角度，要平稳飞行就要目标值为0，目标值遥控器的值
    // 转换数值，remote_data.pitch(0-1000,摇杆默认中间值500)，希望控制角度+_10度
    pitch_pid.desire = (remote_data.pitch - 500) / 50.0;
    // 外环的测量值为当前的俯仰角
    pitch_pid.measure = euler_angle.pitch;
    // 内环的测量值为当前角速度
    gyro_y_pid.measure = (all_data.gyro.gyro_y * 2000.0 / 32768.0);

    // 2.进行pid计算
    Com_pid_calc_chain(&pitch_pid, &gyro_y_pid);
    // 3.输出值
    // debug_printf(":%.2f,%.2f,%.2f\n", pitch_pid.err, pitch_pid.output, gyro_y_pid.output);

    // 横滚角
    // 1.需要目标值和测量值
    // 外环目标角度，平稳飞行，角度为0
    roll_pid.desire = (remote_data.roll - 500) / 50.0;
    // 外环测量值，角度
    roll_pid.measure = euler_angle.roll;
    // 内环测量值
    gyro_x_pid.measure = (all_data.gyro.gyro_x * 2000.0 / 32768.0);

    // 2.计算pid
    Com_pid_calc_chain(&roll_pid, &gyro_x_pid);

    // 偏航角
    //  1.需要目标值和测量值
    yaw_pid.desire = (remote_data.yaw - 500) / 50.0;
    // 外环测量值，角度
    yaw_pid.measure = euler_angle.yaw;
    // 内环测量值
    gyro_z_pid.measure = (all_data.gyro.gyro_z * 2000.0 / 32768.0);

    // 2.计算pid
    Com_pid_calc_chain(&yaw_pid, &gyro_z_pid);
}

/**
 * @brief 根据pid输出值控制电机
 */
void App_flight_control_motor(void)
{
    // 1.判断飞行状态
    switch (flight_state)
    {
    case FLIGHT_IDLE:
        // 加锁状态的话将电机设置为0
        left_top_motor.speed = 0;
        left_bottom_motor.speed = 0;
        right_top_motor.speed = 0;
        right_bottom_motor.speed = 0;

        break;
    case FLIGHT_NORMAL:
        // 俯仰角，往前飞正误差，前面两个电机转的慢，后面两个电机转的快
        left_top_motor.speed = remote_data.thr + gyro_y_pid.output - gyro_x_pid.output + Com_limit(gyro_z_pid.output, 100, -100);
        left_bottom_motor.speed = remote_data.thr - gyro_y_pid.output - gyro_x_pid.output - Com_limit(gyro_z_pid.output, 100, -100);
        right_top_motor.speed = remote_data.thr + gyro_y_pid.output + gyro_x_pid.output - Com_limit(gyro_z_pid.output, 100, -100);
        right_bottom_motor.speed = remote_data.thr - gyro_y_pid.output + gyro_x_pid.output + Com_limit(gyro_z_pid.output, 100, -100);
        break;
    case FLIGHT_FIX_HEIGHT:
        // 只有在定高状态才需要进行pid计算，定高状态也需要平稳飞行
        left_top_motor.speed = remote_data.thr + gyro_y_pid.output - gyro_x_pid.output + Com_limit(gyro_z_pid.output, 100, -100) + fix_height_pid.output;
        left_bottom_motor.speed = remote_data.thr - gyro_y_pid.output - gyro_x_pid.output - Com_limit(gyro_z_pid.output, 100, -100) + fix_height_pid.output;
        right_top_motor.speed = remote_data.thr + gyro_y_pid.output + gyro_x_pid.output - Com_limit(gyro_z_pid.output, 100, -100) + fix_height_pid.output;
        right_bottom_motor.speed = remote_data.thr - gyro_y_pid.output + gyro_x_pid.output + Com_limit(gyro_z_pid.output, 100, -100) + fix_height_pid.output;
        break;
    case FLIGHT_FAIL:
        break;
    default:
        break;
    }

    // 限制电机上限值
    left_top_motor.speed = Com_limit(left_top_motor.speed, 600, 0);
    left_bottom_motor.speed = Com_limit(left_bottom_motor.speed, 600, 0);
    right_top_motor.speed = Com_limit(right_top_motor.speed, 600, 0);
    right_bottom_motor.speed = Com_limit(right_bottom_motor.speed, 600, 0);

    // 安全限制,当油门设置为<50时，强制将速度设置为0
    if (remote_data.thr < 50)
    {
        left_top_motor.speed = 0;
        left_bottom_motor.speed = 0;
        right_top_motor.speed = 0;
        right_bottom_motor.speed = 0;
    }

    // 2.设置点击速度
    Int_Motor_SetSpeed(&left_top_motor);
    Int_Motor_SetSpeed(&left_bottom_motor);
    Int_Motor_SetSpeed(&right_top_motor);
    Int_Motor_SetSpeed(&right_bottom_motor);
}

/**
 * @brief 定高pid处理
 *
 */
void App_flight_fix_height_PID_process(void)
{
    // 24ms一次
    // 1.目标值(按下定高的高度)，测量值(当前高度)
    fix_height_pid.desire = fix_height;
    fix_height_pid.measure = Int_VL53L1X_GetDistance();

    // 2.单环计算pid
    Com_pid_calc(&fix_height_pid);
}
