#ifndef __COM_DEBUG_H
#define __COM_DEBUG_H

#include "usart.h"
#include "stdio.h"

// 如果一直调用打印日志会占用CPU资源，使用宏定义控制是否开启日志输出
#define DEBUG_ENABLE 1

#if DEBUG_ENABLE
// 使用宏定义打印输出bug所在行数和文件名
#define debug_printf(format, ...) printf("["__FILE__     \
                                         ":%d] " format, \
                                         __LINE__, ##__VA_ARGS__)
#else

#define debug_printf(format, ...)

#endif // 日志输出控制

#endif /* __COM_DEBUG_H */
