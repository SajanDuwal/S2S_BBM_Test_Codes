/*
 * com_debug.h
 *
 *  Created on: Feb 6, 2024
 *      Author: sajanduwal
 */

#ifndef INC_COM_DEBUG_H_
#define INC_COM_DEBUG_H_

#include "main.h"

extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef hlpuart1;

void myPrintf(const char *fmt, ...);

void myPrintf2(const char *fmt, ...);

void myPrintf3(const char *fmt, ...);

int bufferSize(char *buffer);

void delay_us(uint32_t us);

#endif /* INC_COM_DEBUG_H_ */
