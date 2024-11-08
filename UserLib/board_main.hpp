/*
 * board_main.hpp
 *
 *  Created on: Nov 2, 2024
 *      Author: gomas
 */

#ifndef BOARD_MAIN_HPP_
#define BOARD_MAIN_HPP_

#include "main.h"
#include "motor.hpp"
#include "as5048a_enc.hpp"

#include "STM32_CommonLib/Math/sin_table.hpp"
#include "STM32_CommonLib/pwm.hpp"
#include "STM32_CommonLib/pid.hpp"
#include "STM32_CommonLib/programable_PWM.hpp"
#include "STM32_CommonLib/LED_pattern.hpp"
#include "STM32_CommonLib/cordic.hpp"
#include "STM32_CommonLib/fdcan_control.hpp"
#include "STM32_CommonLib/Math/filter.hpp"


#include <stdio.h>

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_rx;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

extern UART_HandleTypeDef huart1;

namespace BoardElement{
	inline auto table = SabaneLib::Math::SinTable<10,SabaneLib::Math::TableMode::NORMAL>{};

	namespace PIDIns{
		inline auto position = SabaneLib::PIDBuilder(20000.0f)
				.set_gain(10.0f,0.0f, 0.0f)
				.set_limit(10000.0f)
				.build();
		inline auto speed = SabaneLib::PIDBuilder(20000.0f)
				.set_gain(0.000'1f, 0.000'1f, 0.0f)
				.set_limit(10.0f)
				.build();

		inline auto d_current = SabaneLib::PIBuilder(20000.0f)
				.set_gain(0.02f, 100.0f)
				.set_limit(1.0f)
				.build();

		inline auto q_current = SabaneLib::PIBuilder(20000.0f)
				.set_gain(0.02f, 100.0f)
				.set_limit(1.0f)
				.build();
	}

	inline auto enc = BoardLib::AS5048AState{&hspi1,SS_GPIO_Port,SS_Pin,20000.0f};

	inline auto motor = BoardLib::Motor{
		SabaneLib::PWMHard{&htim1,TIM_CHANNEL_2},
		SabaneLib::PWMHard{&htim1,TIM_CHANNEL_3},
		SabaneLib::PWMHard{&htim1,TIM_CHANNEL_1}
	};

	inline SabaneLib::Math::UVW uvw_i = {.u=0.0f, .v=0.0f, .w=0.0f};
	inline SabaneLib::Math::AB ab_i = {.a = 0.0f, .b = 0.0f};
	inline SabaneLib::Math::DQ dq_i = {.d = 0.0f, .q = 0.0f};

	inline SabaneLib::Math::DQ target_i = {.d = 0.0f, .q =0.0f};

	inline SabaneLib::Math::DQ dq_v;

	inline int32_t atan_enc_bias = 1357;
	inline SabaneLib::Math::UVW uvw_i_bias = {.u=0.0f, .v=0.0f, .w=0.0f};

	inline auto speed_lpf = SabaneLib::Math::LowpassFilter<float>{0.1};

	inline float target_speed = 0.0f;

	inline q15_t angle = 0;
	inline q15_t e_angle = 0;

}

#endif /* BOARD_MAIN_HPP_ */
