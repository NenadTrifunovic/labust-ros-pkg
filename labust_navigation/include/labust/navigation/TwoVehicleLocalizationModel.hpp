/*********************************************************************
 * TwoVehicleLocalizationModel.hpp
 *
 *  Created on: Feb 25, 2013
 *      Author: Dula Nad
 *
 *  Modified on: Sep 22, 2015
 *  	Author: Filip Mandic
 *
 ********************************************************************/

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015-2016, LABUST, UNIZG-FER
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
#ifndef EKF_3DMODEL_HPP_
#define EKF_3DMODEL_HPP_
#include <labust/navigation/SSModel.hpp>
#include <labust/math/NumberManipulation.hpp>


namespace labust
{
  namespace navigation
  {
    /**
     * This class implements a EKF filter two vehicle localization using range, bearing, elevation measurements .
     */
    class TwoVehicleLocalizationModel : public SSModel<double>
    {
    	typedef SSModel<double> Base;
    public:
      typedef vector input_type;
      typedef vector output_type;

      struct ModelParams
      {
    	  ModelParams():
    		  alpha(1),
    		  beta(1),
    		  betaa(0){};

    	  ModelParams(double alpha, double beta, double betaa):
    		  alpha(alpha),
    		  beta(beta),
    		  betaa(betaa){}

    	  inline double Beta(double val)
    	  {
    		  return beta + betaa*fabs(val);
    	  }

    	  double alpha, beta, betaa;
      };

      //enum {xp = 0,yp,zp, hdg, psi,u,w,r,xb,yb,zb,psib,ub,wb,rb,stateNum};
      //enum {inputSize = 0};
      //enum {range=stateNum,bearing,elevation,sonar_range,sonar_bearing,camera_range,camera_bearing,camera_psib,measSize};

      enum {xp = 0,yp,zp, hdg, psi,u,w,r,xb,yb,zb,psib,ub,hdgb,stateNum};
      enum {inputSize = 0};
      enum {range=stateNum,bearing,elevation,sonar_range,sonar_bearing,camera_range,camera_bearing,camera_hdgb,measSize};

      /**
       * The default constructor.
       */
      TwoVehicleLocalizationModel();
      /**
       * Generic destructor.
       */
      ~TwoVehicleLocalizationModel();

      /**
       * Perform a prediction step based on the system input.
       *
       * \param u System input.
       */
      void step(const input_type& input);
      /**
       * Calculates the estimated output of the model.
       *
       * \param y Inserts the estimated output values here.
       */
      void estimate_y(output_type& y);
      /**
       * Initialize the model to default values
       */
      void initModel();

      /**
       * Setup the measurement matrix for available measurements.
       */
      const output_type& update(vector& measurements, vector& newMeas);

      /**
       * Set the model parameters.
       */
      void setParameters(const ModelParams& surge,
    		  const ModelParams& sway,
    		  const ModelParams& heave,
    		  const ModelParams& roll,
    		  const ModelParams& pitch,
    		  const ModelParams& yaw)
      {
    	  this->surge = surge;
    	  this->sway = sway;
    	  this->heave = heave;
    	  this->roll = roll;
    	  this->pitch = pitch;
    	  this->yaw = yaw;
      }

      void calculateXYInovationVariance(const matrix& P, double& xin,double &yin);
      void calculateUVInovationVariance(const matrix& P, double& uin,double &vin);
      double calculateAltInovationVariance(const matrix& P);

      /**
       * Return the speeds in the local frame.
       */
      inline void getNEDSpeed(double& xdot, double& ydot)
      {
      	xdot = this->xdot;
      	ydot = this->ydot;
      }

      inline void useDvlModel(int flag){this->dvlModel = flag;};

    protected:
     /**
       * Calculate the Jacobian matrices.
       */
      void derivativeAW();
      /**
        * Calculate the Jacobian matrices.
        */
       void derivativeHV(int num);
       /**
        * Calculate the nonlinear H derivative.
        */
       void derivativeH();
      /**
       * The model parameters.
       */
      ModelParams surge,sway,heave,roll,pitch,yaw;
      /**
       * The newest measurement.
       */
      output_type measurement;
      /**
       * The NED speeds.
       */
      double xdot,ydot;
      /**
       * The DVL linear/nonlinear flag.
       */
      int dvlModel;
      /**
       * The nonlinear H.
       */
      matrix Hnl;
      /**
       * The nonlinear and final y.
       */
      vector ynl,y;
		/**
		 *  Bearing unwrapper.
		 */

		labust::math::unwrap bearing_unwrap;
		labust::math::unwrap camera_bearing_unwrap;
		labust::math::unwrap sonar_bearing_unwrap;
public:
		int bearing_wrap_index;
    };
  }
}

/* EKF_3DMODEL_HPP_ */
#endif
