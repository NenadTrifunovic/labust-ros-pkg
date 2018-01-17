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
#ifndef LABUST_SIM_ENVIRONMENTMODEL_H
#define LABUST_SIM_ENVIRONMENTMODEL_H
#include <labust/simulation/matrixfwd.hpp>
#include <map>

namespace labust
{
  namespace simulation
  {
  	///The class assembles the rigid body states.
  	struct RBStates
		{
  		RBStates():
  			eta(vector::Zero()),
				nu(vector::Zero()),
				nuacc(vector::Zero()),
				altitude(0){};

  		//Position and orientation
  		vector eta;
  		//Linear and angular velocity
  		vector nu;
  		//Linear and angular acceleration
  		vector nuacc;
  		//The altitude
  		double altitude;
		};

  	/**
     *  This class implements a basic environment model needed by some sensors and
     *  the rigid body simulator.
     */
    struct EnvironmentModel
    {
  		EnvironmentModel():
  			bottom_depth(0),
				currents(vector3::Zero()),
				wave_height(0),
				Ts(0.1){};
    	///Average bottom depth at current vehicle position
    	double bottom_depth;
    	///Currents at the current location
    	vector3 currents;
    	///Average wave height at current location
    	double wave_height;
    	///Other agents
    	std::map<std::string, RBStates> agents;
  		//The used sampling time
  		double Ts;
    };
  }
}

/* LABUST_SIM_ENVIRONMENTMODEL_H */
#endif
