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
*  Copyright (c) 2014, LABUST, UNIZG-FER
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

/*********************************************************************
 *** Common global variables
 ********************************************************************/

enum {X = 0, Y, Z, T};

enum {none = 0,
		placeholder,
		go2point_FA,
		go2point_UA,
		go2point_FA_hdg,
		dynamic_positioning,
		course_keeping_FA,
		course_keeping_UA,
		course_keeping_FA_hdg,
		iso,
		path_following,
		pointer,
		primitiveNum};

const char *PRIMITIVES[] = {"none",
								"placeholder",
								"go2point_FA",
								"go2point_UA",
								"go2point_FA_hdg",
								"dynamic_positioning",
								"course_keeping_FA",
								"course_keeping_UA",
								"course_keeping_FA_hdg",
								"iso",
								"path_following",
								"pointer"};

enum {u=0, v, w, r, x, y, z, psi, x_var, y_var, z_var, psi_var, alt, stateHatNum}; /* Enumeration used for DataManager */
const char *stateVarNames[] = {"u", "v", "w", "r", "x", "y", "z", "psi", "x_var", "y_var", "z_var", "psi_var", "alt"};

const char *pl_placeholder[] = {"\0"};
const char *pl_go2point_FA[] = {"north","east","speed","victory_radius","\0"};
const char *pl_go2point_UA[] = {"north","east","speed","victory_radius","\0"};
const char *pl_go2point_FA_hdg[] = {"north","east","heading","speed","victory_radius","\0"};
const char *pl_dynamic_positioning[] = {"north","east","heading","timeout","\0"};
const char *pl_course_keeping_FA[] = {"course","speed","timeout","\0"};
const char *pl_course_keeping_FA_hdg[] = {"course","speed","heading","timeout","\0"};
const char *pl_iso[] = {"dof","command","hysteresis","reference","sampling_rate","\0"};
const char *pl_path_following[] = {"point","\0"};
const char *pl_pointer[] = {"radius_topic","center_topic","target_topic","\0"};
const char *pl_course_keeping_UA[] = {"course","speed","timeout","\0"};

struct PrimitiveParams{
	PrimitiveParams(){

    	std::vector<std::string> tmp;
		std::string tmp_str;
		int i = 0;
		 for(i = 0; strcmp(pl_go2point_FA[i],"\0") != 0; i++){
			 tmp_str.assign(pl_go2point_FA[i]);
			 tmp.push_back(tmp_str);
		 }
		primitive_params.insert(std::pair<int,std::vector<std::string> >(go2point_FA,tmp));

		tmp.clear();
		 for(i = 0; strcmp(pl_go2point_UA[i],"\0") != 0; i++){
			 tmp_str.assign(pl_go2point_UA[i]);
			 tmp.push_back(tmp_str);
		 }
		primitive_params.insert(std::pair<int,std::vector<std::string> >(go2point_UA,tmp));

		tmp.clear();
		 for(i = 0; strcmp(pl_go2point_FA_hdg[i],"\0") != 0; i++){
			 tmp_str.assign(pl_go2point_FA_hdg[i]);
			 tmp.push_back(tmp_str);
		 }
		primitive_params.insert(std::pair<int,std::vector<std::string> >(go2point_FA_hdg,tmp));

		tmp.clear();
		 for(i = 0; strcmp(pl_dynamic_positioning[i],"\0") !=0; i++){
			 tmp_str.assign(pl_dynamic_positioning[i]);
			 tmp.push_back(tmp_str);
		 }
		primitive_params.insert(std::pair<int,std::vector<std::string> >(dynamic_positioning,tmp));

		tmp.clear();
		 for(i = 0; strcmp(pl_course_keeping_FA[i],"\0") != 0; i++){
			 tmp_str.assign(pl_course_keeping_FA[i]);
			 tmp.push_back(tmp_str);
		 }
		primitive_params.insert(std::pair<int,std::vector<std::string> >(course_keeping_FA,tmp));

		tmp.clear();
		 for(i = 0; strcmp(pl_course_keeping_UA[i],"\0") != 0; i++){
			 tmp_str.assign(pl_course_keeping_UA[i]);
			 tmp.push_back(tmp_str);
		 }

		 primitive_params.insert(std::pair<int,std::vector<std::string> >(course_keeping_UA,tmp));

		tmp.clear();
		 for(i = 0; strcmp(pl_iso[i],"\0") != 0; i++){
			 tmp_str.assign(pl_iso[i]);
			 tmp.push_back(tmp_str);
		 }
		 primitive_params.insert(std::pair<int,std::vector<std::string> >(iso,tmp));

		tmp.clear();
		 for(i = 0; strcmp(pl_path_following[i],"\0") != 0; i++){
			 tmp_str.assign(pl_path_following[i]);
			 tmp.push_back(tmp_str);
		 }
		primitive_params.insert(std::pair<int,std::vector<std::string> >(path_following,tmp));

		tmp.clear();
		 for(i = 0; strcmp(pl_iso[i],"\0") != 0; i++){
			 tmp_str.assign(pl_pointer[i]);
			 tmp.push_back(tmp_str);
		 }
		primitive_params.insert(std::pair<int,std::vector<std::string> >(pointer,tmp));

		tmp.clear();
		 for(int i = 0; strcmp(pl_placeholder[i],"\0") != 0; i++){
			 tmp_str.assign(pl_placeholder[i]);
			 tmp.push_back(tmp_str);
		 }
		primitive_params[placeholder] = tmp;

	}

	~PrimitiveParams(){}

	std::map<int,  std::vector<std::string> > primitive_params;
};

using namespace std;

#endif /* LABUSTMISSION_HPP_ */
