//TODO Add status merger
/*
 * StatusHandlers.h
 *
 *  Created on: Jul 18, 2016
 *      Author: filip
 */

#ifndef LABUST_ROS_PKG_LABUST_DIAGNOSTICS_INCLUDE_LABUST_DIAGNOSTICS_STATUSHANDLER_H_
#define LABUST_ROS_PKG_LABUST_DIAGNOSTICS_INCLUDE_LABUST_DIAGNOSTICS_STATUSHANDLER_H_

#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <labust/diagnostics/StatusHandlerBase.h>

namespace labust
{
	namespace diagnostic
	{

		class StatusHandler: public StatusHandlerBase
		{
		public:
			StatusHandler(const std::string& entity_name, const std::string& entity_id)
			{
				this->setEntityName(entity_name);
				this->setEntityId(entity_id);

				ros::NodeHandle nh;
				pub_status_ = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics/"+entity_id,1);
			}

			~StatusHandler()
			{

			}

			void publishStatus()
			{
				diagnostic_msgs::DiagnosticArray::Ptr diagnostic_array(new diagnostic_msgs::DiagnosticArray);
				diagnostic_array->header.stamp = ros::Time::now();
				diagnostic_array->status.push_back(this->entity_status_);
				pub_status_.publish(diagnostic_array);
			}

			ros::Publisher pub_status_;
		};
	}
}

#endif /* LABUST_ROS_PKG_LABUST_DIAGNOSTICS_INCLUDE_LABUST_DIAGNOSTICS_STATUSHANDLER_H_ */
