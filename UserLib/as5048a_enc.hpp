/*
 * as5048a_enc.hpp
 *
 *  Created on: Nov 2, 2024
 *      Author: gomas
 */

#ifndef AS5048A_ENC_HPP_
#define AS5048A_ENC_HPP_

#include "STM32_CommonLib/encoder.hpp"

#include "main.h"

namespace BoardLib{
	class AS5048AState:public SabaneLib::ContinuableEncoder{
		static constexpr size_t as5048a_resolution = 12;
		static constexpr size_t mask = (1<<as5048a_resolution) -1;

		SPI_HandleTypeDef* spi;
		GPIO_TypeDef *port;
		const uint32_t pin;

		volatile uint8_t enc_val[2] = {0};
	public:
		AS5048AState(SPI_HandleTypeDef* _spi,GPIO_TypeDef *_port,uint32_t _pin,float _freq,bool is_inv = false)
			:ContinuableEncoder(as5048a_resolution,_freq),
			 spi(_spi),
			 port(_port),
			 pin(_pin){
		}

		void start(void){
			LL_GPIO_SetOutputPin(port,pin);
		}

		void read_start(void){
			uint8_t tx[2] = {0xFF,0xFF};
			LL_GPIO_ResetOutputPin(port,pin);
			HAL_SPI_TransmitReceive_IT(spi,tx,const_cast<uint8_t*>(enc_val), 2);
		}

		void spi_rx_interrupt_task(void){
			LL_GPIO_SetOutputPin(port,pin);
			uint16_t raw_angle = ((enc_val[0]<<8) | enc_val[1])&mask;
			update(raw_angle);

		}
	};
}



#endif /* AS5048A_ENC_HPP_ */
