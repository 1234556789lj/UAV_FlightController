#ifndef __INT_LED_H
#define __INT_LED_H

#include "main.h"

// 使用结构体封装LED数据
typedef struct
{
    GPIO_TypeDef *port; // GPIO端口
    uint16_t pin;       // GPIO引脚

} LED_St;

void Int_LED_On(LED_St *led);
void Int_LED_Off(LED_St *led);
void Int_LED_Toggle(LED_St *led);

#endif /* __INT_LED_H */
