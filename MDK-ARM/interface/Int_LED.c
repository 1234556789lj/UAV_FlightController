#include "Int_LED.h"

/*
 ******************************************************************************
 * @file           : Int_LED.c
 * @brief          : LED点亮LED，低电平点亮，高电平熄灭，翻转LED状态
 ******************************************************************************
 */
void Int_LED_On(LED_St *led)
{
    HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_RESET);
}

void Int_LED_Off(LED_St *led)
{
    HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_SET);
}

void Int_LED_Toggle(LED_St *led)
{
    HAL_GPIO_TogglePin(led->port, led->pin);
}
