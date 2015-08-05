/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2015, LABUST, UNIZG-FER
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the LABUST nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#ifndef ALLOCATION_ETHRUSTER_H
#define ALLOCATION_ETHRUSTER_H
#include <cmath>

namespace labust
{
	namespace allocation
	{
		/**
		 * The class implements a object representation of a single
		 * electrical thruster with the standard quadratic
		 * characteristics.
		 *
		 * F = K*Ua*|Ua| = Kn*Us^2*p|p|
		 * where:
		 * Ua - general Ua voltage
		 * Us - current supply voltage (Us>0)
		 * K - the thruster gain (Kp for positive, Kn for negative); K,Kp,Kn>0
		 * p - PWM width [-1,1]
		 * F - the supplied thruster force
		 */
		class EThruster
		{
		public:
			///Main constructor
			EThruster():
				Kp(1),
				Kn(1),
				pwm_id(-1),
				pwm_max(1.0),
				pwm_min(-1.0),
				pwm_dir(1){};

			///Init constructor
			EThruster(double Kp, double Kn, int pwm_id,
					double pwm_max, double pwm_min, double pwm_dir):
				Kp(Kp),
				Kn(Kn),
				pwm_id(pwm_id),
				pwm_max(pwm_max),
				pwm_min(pwm_min),
				pwm_dir(pwm_dir){};

			/**
			 * Calculate the force given a supply voltage and PWM value.
			 * @param Us - the supply voltage (default: 1)
			 * @param p - the PWM values (default: 1)
			 */
			inline double F(double p = 1, double Us=1){return (p>=0)?Kp*Us*Us*p*p:-Kn*Us*Us*p*p;}
			/**
			 * Calculate the required pwm value given a desired force.
			 * @param Us - the supply voltage (default: 1)
			 * @param p - the PWM values (default: 1)
			 * \return
			 */
			inline double pwm(double F, double Us=1)
			{
				if ((Kp == 0) || (Kn == 0) || (Us == 0)) return 0.0;
				return (F>=0)?sqrt(F/Kp)/Us:-std::sqrt(-F/Kn)/Us;
			}

			///The Kn+ gain for positive voltages.
			double Kp;
			///The Kn- gain for negative voltages.
			double Kn;
			///The connected PWM id.
			int pwm_id;
			///The maximum pwm
			double pwm_max;
			///The minimum pwm
			double pwm_min;
			///The pwm corrected direction
			int pwm_dir;
		};
	}
}

/* ALLOCATION_ETHRUSTER_H */
#endif
