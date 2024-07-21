/*
 * ESP8266.h
 *
 *  Created on: 25 лип. 2023 р.
 *      Author: Asuka
 */
#include "stm32f4xx_hal.h"
#include "stm32f401xc.h"

#ifndef INC_ESP8266_H_
#define INC_ESP8266_H_

#define MAX_BUFFER_SIZE 100
extern char buffer[MAX_BUFFER_SIZE];
extern volatile uint32_t buffer_index;

void USARTinit();

#endif /* INC_ESP8266_H_ */
