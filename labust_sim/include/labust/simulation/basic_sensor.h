/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, LABUST, UNIZG-FER
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
#ifndef LABUST_SIM_BASICSENSOR_H
#define LABUST_SIM_BASICSENSOR_H
#include <labust/simulation/matrixfwd.hpp>
#include <labust/simulation/environment_model.h>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/nondet_random.hpp>
#include <boost/shared_ptr.hpp>

namespace labust
{
  namespace simulation
  {
  	/**
     *  This class implements the simulated sensor interface that incorporates main
     *  elements of each sensor as relative position on the vehicle and noise parameters.
     *  TODO: revisit and check noise generation
     *  TODO: add environmental model (contains:
     */
    class BasicSensor
    {
    public:
    	///Random generator type shortcut
			typedef boost::variate_generator<boost::random_device&, boost::normal_distribution<> > NoiseGenerator;
			///Random generator pointer
			typedef boost::shared_ptr<NoiseGenerator> NoiseGeneratorPtr;

			///The mapping enumerator
			enum {m_x,m_y,m_z,m_phi,m_theta,m_psi,
				m_u,m_v,m_w,m_p,m_q,m_r,
				m_uacc,m_vacc,m_wacc,m_pacc,m_qacc,m_racc,
				m_alt
			};

      ///Default constructor
      BasicSensor():
      	offset(vector3::Zero()),
				orot(1,0,0,0){};
      ///Default virtual destructor
      virtual ~BasicSensor(){};

      /**
       * The method performs one simulation step. Ideal measurements are passed to the sensor
       * to emulate a realistic sensor measurement.
       *
       * \param eta Position and orientation.
       * \param nu Linear and angular speeds.
       * \param nuacc Linear and angular accelerations
       * \param env The environment model at the current location.
       */
      virtual void step(const vector& eta,
      		const vector& nu,
					const vector& nuacc,
					const EnvironmentModel& env) = 0;

      ///The method returns simulated measurements
      const Eigen::VectorXd& getSimMeasurement(){return sim_meas;};
      /**
       * The method returns the measurement mapping. The measurement value in sim_meas[i]
       * maps to the state map_meas[i]. Where the states are defined as:
       * 0-5 (eta), 6-11 (nu), 12-17 (nuacc), 18 (altitude)
       */
      const Eigen::VectorXd& getMeasurementMapping(){return map_meas;};

      /**
       * The method configures measurement noise.
       *
       * \param mean The measurement noise mean.
       * \param var The mesurement noise variance.
       *
       * \return True if successful, False otherwise.
       */
      virtual bool setNoise(const Eigen::VectorXd& mean, const Eigen::VectorXd& var) = 0;

      /**
       * The method configures the sensor offsets relative to the vehicle center.
       *
       * \param offset Positional offset
       * \param orot The angular offset in quaternion form
       */
      void offsets(const vector3& offset, const quaternion& orot)
      {
      	this->offset = offset;
      	this->orot = orot;
      }

    protected:
      ///The sensor positional offset
      vector3 offset;
      ///The sensor rotation quaternion
      quaternion orot;
      ///The simulated measurement vector
      Eigen::VectorXd sim_meas;
      ///The measurement map
      Eigen::VectorXd map_meas;
      ///Random number generator
      boost::random_device rd;
    };
  }
}

/* LABUST_SIM_BASICSENSOR_H */
#endif
