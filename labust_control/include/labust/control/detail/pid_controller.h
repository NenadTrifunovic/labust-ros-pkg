/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, LABUST, UNIZG-FER
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
#ifndef LABUST_CONTROL_PID_CONTROLLER_H
#define LABUST_CONTROL_PID_CONTROLLER_H

#include <labust/control/detail/proportional.h>
#include <labust/control/detail/integral.h>
#include <labust/control/detail/derivative.h>

#include <limits>

namespace labust
{
  namespace control
  {
    namespace detail
    {
      /**
       * The class implements a parallel PID controller.
       * The windup is handled by conditional integration.
       */
      class ParallelPID
      {
      public:
        /// Generic constructor
        ParallelPID():
          dT(0.1),
          windup(false),
          refk_1(0.0),
          statek_1(0.0),
          max(std::numeric_limits<double>::max()),
          min(-std::numeric_limits<double>::min()){};

        /// Dummy calculation step
        inline double step()
        {
          return output;
        }

        void exwindup(bool windup, double track)
        {
          // Update the windup if windup was detected externally
          this->windup = windup || this->windup;
        }

        /// Main calculation step
        double step(double ref, double state, double ff = 0, double fl = 0)
        {
          // Update derivative and integral term
          if (!windup)
          {
            I.step(refk_1, statek_1);
          }
          D.step(refk_1, statek_1);

          // Calculate output
          double vk = P.step(ref, state) + I.step() + D.step();
          vk += ff + fl;

          uk = sat(vk);

          // Check if windup occured
          windup = ((vk*(ref-state) > 0) && (uk != vk));

          // Store variables
          refk_1 = ref;
          statek_1 = state;
        }

        /// The proportional part of PID.
        Proportional P;
        /// The integral part of PID.
        Integral I;
        /// The derivative gain
        Derivative D;
        /// The sampling time
        double dT;
        /// The windup flag
        bool windup;
        /// The reference from the previous step.
        double refk_1;
        /// The state from the previous step.
        double statek_1;
        /// The maximum output
        double max;
        /// The minimum output
        double min;
        /// External windup
        bool ewindup;

        inline double sat(double value)
        {
          if (value > max)
            return max;
          else if (value < min)
            return min;

          return value;
        }

      private:
        /// The final calculate output
        double uk;
      };
    }
  }
}

// LABUST_CONTROL_PID_CONTROLLER_H
#endif
