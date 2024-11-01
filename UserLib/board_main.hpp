/*
 * board_main.hpp
 *
 *  Created on: Nov 2, 2024
 *      Author: gomas
 */

#ifndef BOARD_MAIN_HPP_
#define BOARD_MAIN_HPP_

#include "main.h"
#include <stdio.h>

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_rx;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

extern UART_HandleTypeDef huart1;


#endif /* BOARD_MAIN_HPP_ */
