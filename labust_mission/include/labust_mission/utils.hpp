/*********************************************************************
 * utils.hpp
 *
 *  Created on: Jun 16, 2014
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

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <ros/ros.h>

namespace labust{
	namespace utilities{

		/*************************************************************
		 ***  Functions used calling ROS services
		 *************************************************************/
		template <typename custom_srv>
		custom_srv callService(ros::ServiceClient& client, custom_srv& request){

			if (client.call(request)){
				//ROS_INFO("Call to service %s successful", client.getService().c_str());
				return request;
			} else {
				ROS_ERROR("Call to service %s failed", client.getService().c_str());
			}
		}

		/*************************************************************
		 ***  Functions for ROS message serialization and deserialization
		 *************************************************************/
		template <typename msgType>
		std::vector<uint8_t> serializeMsg(msgType data){

			uint32_t serial_size = ros::serialization::serializationLength(data);
			std::vector<uint8_t> buffer(serial_size);

			ros::serialization::OStream stream(&buffer.front(), serial_size);
			ros::serialization::serialize(stream, data);

			return buffer;
		}

		template <typename msgType>
		msgType deserializeMsg(std::vector<uint8_t> my_buffer){

			msgType my_msg;

			uint32_t serial_size = ros::serialization::serializationLength(my_msg);
			uint8_t *iter = &my_buffer.front();

			ros::serialization::IStream stream(iter, serial_size);
			ros::serialization::deserialize(stream, my_msg);

			return my_msg;
		}

		/*************************************************************
		 ***  Functions used for splitting string expressions
		 *************************************************************/
		std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
			std::stringstream ss(s);
			std::string item;
			while (std::getline(ss, item, delim)) {
				if(!item.empty()){
					elems.push_back(item);
				}
			}
			return elems;
		}

		std::vector<std::string> split(const std::string &s, char delim) {
			std::vector<std::string> elems;
			split(s, delim, elems);
			return elems;
		}
	}
}

#endif /* UTILS_HPP_ */
