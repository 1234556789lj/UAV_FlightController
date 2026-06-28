#include "Int_IP5305T.h"

void Int_IP5305T_Start(void)
{
    HAL_GPIO_WritePin(power_key_GPIO_Port, power_key_Pin, GPIO_PIN_RESET);
    vTaskDelay(100);
    HAL_GPIO_WritePin(power_key_GPIO_Port, power_key_Pin, GPIO_PIN_SET);
}

void Int_IP5305T_Shutdown(void)
{
    HAL_GPIO_WritePin(power_key_GPIO_Port, power_key_Pin, GPIO_PIN_RESET);
    vTaskDelay(100);
    HAL_GPIO_WritePin(power_key_GPIO_Port, power_key_Pin, GPIO_PIN_SET);
    vTaskDelay(200);
    HAL_GPIO_WritePin(power_key_GPIO_Port, power_key_Pin, GPIO_PIN_RESET);
    vTaskDelay(100);
    HAL_GPIO_WritePin(power_key_GPIO_Port, power_key_Pin, GPIO_PIN_SET);
}
