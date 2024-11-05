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

		void move(float power, SabaneLib::Math::SinCos sc){
			SabaneLib::Math::UVW tmp = SabaneLib::Math::DQ{0, power}.to_uvw(sc);
			u(tmp.u*0.4f + 0.5f);
			v(tmp.v*0.4f + 0.5f);
			w(tmp.w*0.4f + 0.5f);
		}

		void move(SabaneLib::Math::DQ dq_v, SabaneLib::Math::SinCos sc){
			SabaneLib::Math::UVW tmp = dq_v.to_uvw(sc);
			u(tmp.u*0.4f + 0.5f);
			v(tmp.v*0.4f + 0.5f);
			w(tmp.w*0.4f + 0.5f);
		}

		void move(SabaneLib::Math::UVW uvw_v){
			u(uvw_v.u*0.4f + 0.5f);
			v(uvw_v.v*0.4f + 0.5f);
			w(uvw_v.w*0.4f + 0.5f);
		}
	};
}



#endif /* MOTOR_HPP_ */
