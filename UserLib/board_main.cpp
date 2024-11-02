/*
 * board_main.cpp
 *
 *  Created on: Nov 2, 2024
 *      Author: gomas
 */



#include "board_main.hpp"

namespace b = BoardElement;
namespace blib = BoardLib;
namespace slib = SabaneLib;

extern "C" int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart1, (uint8_t*) ptr, len,100);
	return len;
}


volatile uint8_t rx_buff[2] = {0};

extern "C" void main_(void){
	LL_GPIO_SetOutputPin(SS_GPIO_Port,SS_Pin);

	b::table.generate();

	b::PIDIns::d_current.set_limit(0.0f);
	b::PIDIns::q_current.set_limit(0.0f);

	b::motor.start();

	HAL_ADC_Start(&hadc1);
	HAL_ADCEx_InjectedStart_IT(&hadc1);
	HAL_ADC_Start(&hadc2);
	HAL_ADCEx_InjectedStart_IT(&hadc2);

	HAL_Delay(1000);

	SabaneLib::MotorMath::UVW tmp = {.u=0.0f, .v=0.0f, .w=0.0f};
	for(int i = 0; i < 1000; i++){
		tmp.u += b::uvw_i.u;
		tmp.v += b::uvw_i.v;
		tmp.w += b::uvw_i.w;
		HAL_Delay(1);
	}

	b::uvw_i_bias.u = tmp.u/1000.0f;
	b::uvw_i_bias.v = tmp.v/1000.0f;
	b::uvw_i_bias.w = tmp.w/1000.0f;

	b::PIDIns::d_current.set_limit(0.5f);
	b::PIDIns::q_current.set_limit(0.5f);

	HAL_TIM_Base_Start_IT(&htim2);

	while(1){
//		b::angle += 100;
//		b::motor.move(0.2,b::table.sin_cos(b::angle));

//		b::motor.u.out(0.5);
//		b::motor.v.out(0.45);
//		b::motor.w.out(0.45);

//		printf("%4.3f,%4.3f,%4.3f,%4.3f,%4.3f,%4.3f\r\n",
//				b::uvw_i.u,
//				b::uvw_i.v,
//				b::uvw_i.w,
//				b::motor.u.get(),
//				b::motor.v.get(),
//				b::motor.w.get());

//		printf("%4.3f,%4.3f,%4.3f,%4.3f,%4.3f\r\n",
//				b::uvw_i.u,
//				b::uvw_i.v,
//				b::uvw_i.w,
//				b::dq_i.d,
//				b::dq_i.q);

//		printf("%4.3f,%4.3f,%4.3f,%4.3f,%4.3f,%4.3f\r\n",
//				b::dq_v.d,
//				b::dq_v.q,
//				b::dq_i.d,
//				b::dq_i.q,
//				b::target_i.d,
//				b::target_i.q);

//		printf("%d,%d,%d\r\n",
//				b::e_angle,
//				-b::enc.get_angle(),
//				b::speed_lpf(b::enc.get_speed())/100);

		printf("%4.3f,%4.3f\r\n",
				b::target_i.q,
				b::speed_lpf(b::enc.get_speed())/100);

		HAL_Delay(10);

	}
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc){

	static int adc_flag = 0;
	if(hadc == &hadc1){

	}else if(hadc == &hadc2){
		LL_GPIO_SetOutputPin(LED_GPIO_Port,LED_Pin);
		b::enc.read_start();

		constexpr float adc_to_current = 0.025;//3.3f/static_cast<float>(0xFFF);
		b::uvw_i.v = -static_cast<float>(ADC2->JDR1)*adc_to_current - b::uvw_i_bias.v;
		b::uvw_i.w =  static_cast<float>(ADC2->JDR2)*adc_to_current - b::uvw_i_bias.w;
		b::uvw_i.u = -b::uvw_i.w - b::uvw_i.v;

		LL_GPIO_ResetOutputPin(LED_GPIO_Port,LED_Pin);
	}
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
	b::enc.spi_rx_interrupt_task();

	//現在の電気角を取得
	b::e_angle = ((-b::enc.get_angle()&0x7FF)<<5) - 4800;

	//電流のUVW->DQ
	auto sc = b::table.sin_cos(b::e_angle);
	b::ab_i = b::uvw_i.to_ab();
	b::dq_i = b::ab_i.to_dq(sc);

//	b::target_i = {0.0f,0.1f};

	b::dq_v = slib::MotorMath::DQ{
			.d = b::PIDIns::d_current(b::target_i.d,b::dq_i.d),
			.q = b::PIDIns::q_current(b::target_i.q,b::dq_i.q)};

	b::motor.move(b::dq_v.to_uvw(sc));
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim2){
		//位置制御
//		LL_GPIO_SetOutputPin(LED_GPIO_Port,LED_Pin);
		b::target_i.d = 0.0f;
		b::target_i.q = b::PIDIns::speed(20000, -b::enc.get_speed());
//		LL_GPIO_ResetOutputPin(LED_GPIO_Port,LED_Pin);
	}
}
