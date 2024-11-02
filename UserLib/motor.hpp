/*
 * motor.hpp
 *
 *  Created on: Nov 2, 2024
 *      Author: gomas
 */

#ifndef MOTOR_HPP_
#define MOTOR_HPP_

#include "main.h"

#include "STM32_CommonLib/pwm.hpp"
#include "STM32_CommonLib/Math/motor_math.hpp"
#include "STM32_CommonLib/encoder.hpp"

#include <functional>

namespace s = SabaneLib;

namespace BoardLib{
	class Motor{


	public:
		s::PWMHard u,v,w;
		Motor(s::PWMHard _u,s::PWMHard _v, s::PWMHard _w)
			:u(_u),
			 v(_v),
			 w(_w){
		}

		void start(void){
			u.start();
			v.start();
			w.start();
		}

		void move(float power, SabaneLib::MotorMath::SinCos sc){
			SabaneLib::MotorMath::UVW tmp = SabaneLib::MotorMath::DQ{0, power}.to_uvw(sc);
			u.out(tmp.u*0.4f + 0.5f);
			v.out(tmp.v*0.4f + 0.5f);
			w.out(tmp.w*0.4f + 0.5f);
		}

		void move(SabaneLib::MotorMath::DQ dq_v, SabaneLib::MotorMath::SinCos sc){
			SabaneLib::MotorMath::UVW tmp = dq_v.to_uvw(sc);
			u.out(tmp.u*0.4f + 0.5f);
			v.out(tmp.v*0.4f + 0.5f);
			w.out(tmp.w*0.4f + 0.5f);
		}

		void move(SabaneLib::MotorMath::UVW uvw_v){
			u.out(uvw_v.u*0.4f + 0.5f);
			v.out(uvw_v.v*0.4f + 0.5f);
			w.out(uvw_v.w*0.4f + 0.5f);
		}
	};
}



#endif /* MOTOR_HPP_ */