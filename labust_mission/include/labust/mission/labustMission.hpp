/*********************************************************************
 * labustMission.hpp
 *
 *  Created on: Apr 10, 2014
 *      Author: Filip Mandic
 *
 ********************************************************************/

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014-2016, LABUST, UNIZG-FER
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

#ifndef LABUSTMISSION_HPP_
#define LABUSTMISSION_HPP_

/*********************************************************************
 *** Common includes
 ********************************************************************/

#include <string>

#include <ros/ros.h>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <labust_mission/utils.hpp>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64MultiArray.h>

#include <misc_msgs/SendPrimitive.h>
#include <misc_msgs/Go2PointFA.h>
#include <misc_msgs/Go2PointUA.h>
#include <misc_msgs/DynamicPositioning.h>
#include <misc_msgs/CourseKeepingFA.h>
#include <misc_msgs/CourseKeepingUA.h>
#include <misc_msgs/ISO.h>
#include <misc_msgs/ExternalEvent.h>
#include <misc_msgs/MissionSetup.h>
#include <misc_msgs/DataEventsContainer.h>
#include <misc_msgs/EvaluateExpression.h>

#include <auv_msgs/NED.h>
#include <auv_msgs/NavSts.h>
#include <auv_msgs/DecimalLatLon.h>

/*********************************************************************
 *** Common global variables
 ********************************************************************/

enum {X = 0, Y, Z, T};

enum {none = 0,
		placeholder,
		go2point,
		dynamic_positioning,
		course_keeping,
		pointer,
		iso,
		follow,
		docking,
		primitiveNum};

const char *PRIMITIVES[] = {"none",
								"placeholder",
								"go2point",
								"dynamic_positioning",
								"course_keeping",
								"pointer",
								"iso",
								"follow",
								"docking"};

enum {u=0, v, w, r, x, y, z, psi, x_var, y_var, z_var, psi_var, alt, stateHatNum}; /* Enumeration used for DataManager */
const char *stateVarNames[] = {"u", "v", "w", "r", "x", "y", "z", "psi", "x_var", "y_var", "z_var", "psi_var", "alt"};

using namespace std;

#endif /* LABUSTMISSION_HPP_ */
