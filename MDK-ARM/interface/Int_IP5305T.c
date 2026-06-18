#include "Int_IP5305T.h"

// 启动电源
void Int_IP5305T_Start(void)
{
    // 短按（30ms-2s）：打开电量显示灯和升压输出
    HAL_GPIO_WritePin(power_key_GPIO_Port, power_key_Pin, GPIO_PIN_RESET);
    vTaskDelay(100);
    HAL_GPIO_WritePin(power_key_GPIO_Port, power_key_Pin, GPIO_PIN_SET);
}

// 1s内两次短按关闭电源
void Int_IP5305T_Shutdown(void)
{
    // 关闭电源
    HAL_GPIO_WritePin(power_key_GPIO_Port, power_key_Pin, GPIO_PIN_RESET);
    vTaskDelay(100);
    HAL_GPIO_WritePin(power_key_GPIO_Port, power_key_Pin, GPIO_PIN_SET);

    vTaskDelay(200);

    HAL_GPIO_WritePin(power_key_GPIO_Port, power_key_Pin, GPIO_PIN_RESET);
    vTaskDelay(100);
    HAL_GPIO_WritePin(power_key_GPIO_Port, power_key_Pin, GPIO_PIN_SET);
}
