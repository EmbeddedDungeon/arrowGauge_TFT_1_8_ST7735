/*
 * SDFlash.h
 *
 *  Created on: 19 лип. 2024 р.
 *      Author: Asuka
 */
#include "stm32f4xx.h"
#include "stm32f401xc.h"
#include "stm32f4xx_it.h"
#ifndef INC_SDFLASH_H_
#define INC_SDFLASH_H_

void SPI2_Init();
void SPI2_SendByte(uint8_t byte);
uint8_t SPI2_ReceiveByte(void);

uint8_t SPI2_TransferByte(uint8_t byte);
char SPI2_LoopbackTest(void);


uint8_t SPI2_CheckSDCard(void);
uint8_t SPI2_IsDataAvailable(void);

#endif /* INC_SDFLASH_H_ */
