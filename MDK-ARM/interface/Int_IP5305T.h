#ifndef INT_IP5305T_H
#define INT_IP5305T_H

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
// 启动电源
void Int_IP5305T_Start(void);

void Int_IP5305T_Shutdown(void);

#endif // INT_IP5305T_H
