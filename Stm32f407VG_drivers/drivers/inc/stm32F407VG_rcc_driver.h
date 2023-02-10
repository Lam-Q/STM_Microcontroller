/*
 * stm32F407VG_rcc_driver.h
 *
 *  Created on: Nov 13, 2022
 *      Author: lamqi
 */

#ifndef INC_STM32F407VG_RCC_DRIVER_H_
#define INC_STM32F407VG_RCC_DRIVER_H_


#include "stm32f407VG.h"


uint32_t RCC_GetPLLOutputClock();




uint32_t RCC_GETPCLK1Value(void);

uint32_t RCC_GETPCLK2Value(void);

#endif /* INC_STM32F407VG_RCC_DRIVER_H_ */
