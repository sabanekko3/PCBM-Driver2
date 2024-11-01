/*
 * board_main.cpp
 *
 *  Created on: Nov 2, 2024
 *      Author: gomas
 */



#include "board_main.hpp"


extern "C" int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart1, (uint8_t*) ptr, len,100);
	return len;
}
volatile uint8_t rx_buff[2] = {0};
extern "C" void main_(void){
	LL_GPIO_SetOutputPin(SS_GPIO_Port,SS_Pin);
	HAL_TIM_Base_Start_IT(&htim2);
	printf("test\r\n");
	while(1){
		printf("%d\r\n",((rx_buff[0]<<8)|rx_buff[1])&0x3FFF);
		HAL_Delay(10);

	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim2){
		//位置制御
		uint8_t tx_data[2]={0xFF,0xFF};
		LL_GPIO_SetOutputPin(SS_GPIO_Port,SS_Pin);

		LL_GPIO_SetOutputPin(LED_GPIO_Port,LED_Pin);
		LL_GPIO_ResetOutputPin(SS_GPIO_Port,SS_Pin);
		HAL_SPI_TransmitReceive_DMA(&hspi1,tx_data, const_cast<uint8_t*>(rx_buff), 2);//Read data

		LL_GPIO_ResetOutputPin(LED_GPIO_Port,LED_Pin);
	}
}
