/*
 * DiagnosticHandler.h
 *
 *  Created on: Jul 18, 2016
 *      Author: filip
 */

#ifndef LABUST_ROS_PKG_LABUST_DIAGNOSTICS_INCLUDE_LABUST_DIAGNOSTICS_DIAGNOSTICHANDLER_H_
#define LABUST_ROS_PKG_LABUST_DIAGNOSTICS_INCLUDE_LABUST_DIAGNOSTICS_DIAGNOSTICHANDLER_H_

#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>

//#include <labust/diagnostics/StatusHandlerBase.h>

/*************************************************************
 ***  Command processor
 *************************************************************/

// C++11 implementation
/*std::string exec(const char* cmd) {
    char buffer[128];
    std::string result = "";
    std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
    if (!pipe) throw std::runtime_error("popen() failed!");
    while (!feof(pipe.get())) {
        if (fgets(buffer, 128, pipe.get()) != NULL)
            result += buffer;
    }
    return result;
}*/

std::string exec(const char* cmd) {
    char buffer[128];
    std::string result = "";
    FILE* pipe = popen(cmd, "r");
    if (!pipe) throw std::runtime_error("popen() failed!");
    try {
        while (!feof(pipe)) {
            if (fgets(buffer, 128, pipe) != NULL)
                result += buffer;
        }
    } catch (...) {
        pclose(pipe);
        throw;
    }
    pclose(pipe);
    return result;
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


namespace labust
{
	namespace diagnostic
	{

		class DiagnosticHandler
		{
		public:
			DiagnosticHandler();

			~DiagnosticHandler()
			{

			}

			void subscribeTopics();

			void onDiagnosticData(const diagnostic_msgs::DiagnosticArray::ConstPtr& data);

			void checkStatus();

			ros::Publisher pub_diagnostic_array_;

			std::vector<ros::Subscriber> sub_diagnostic_topic_;

			std::vector<diagnostic_msgs::DiagnosticArray> diagnostic_array_;

		};

		DiagnosticHandler::DiagnosticHandler()
		{
			ros::NodeHandle nh;
			pub_diagnostic_array_ = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics/",1);

		}

		void DiagnosticHandler::subscribeTopics()
		{
			std::vector<std::string> diagnostic_topics(split(exec("rostopic list"),'\n'));

			for(std::vector<std::string>::iterator it  = diagnostic_topics.begin(); it != diagnostic_topics.end(); ++it)
			{
				std::string diagnostic_ns("/diagnostics/");
				if((*it).find(diagnostic_ns) == std::string::npos)
				{
					diagnostic_topics.erase(it--);
				}
			}

			ros::NodeHandle nh;
			int i = 0;
			for(std::vector<std::string>::const_iterator it  = diagnostic_topics.begin(); it != diagnostic_topics.end(); ++it)
			{
				std::cout << "Subscribing to topic " << (*it) << std::endl;
				sub_diagnostic_topic_.push_back(nh.subscribe<diagnostic_msgs::DiagnosticArray>((*it).c_str(),1,&DiagnosticHandler::onDiagnosticData,this));
			}
		}

		void DiagnosticHandler::onDiagnosticData(const diagnostic_msgs::DiagnosticArray::ConstPtr& data)
		{

			for(std::vector<diagnostic_msgs::DiagnosticArray>::iterator it  = diagnostic_array_.begin(); it != diagnostic_array_.end(); ++it)
			{
				std::string entitity_name((*it).status[0].name);
				if(entitity_name.compare(data->status[0].name))
				{
					(*it).header = data->header;
					(*it).status = data->status;

					pub_diagnostic_array_.publish(*it);

					break;
				}
			}
		}

		void DiagnosticHandler::checkStatus()
		{
			for(std::vector<diagnostic_msgs::DiagnosticArray>::iterator it  = diagnostic_array_.begin(); it != diagnostic_array_.end(); ++it)
			{
				if(ros::Time::now().toSec() - (*it).header.stamp.toSec()>4.0)
				{
					signed char* status = &(it->status[0].level);

					if(*status == diagnostic_msgs::DiagnosticStatus::OK || *status == diagnostic_msgs::DiagnosticStatus::WARN)
					{
						*status = diagnostic_msgs::DiagnosticStatus::STALE;
					}
					pub_diagnostic_array_.publish(*it);
				}
			}
		}
	}
}



#endif /* LABUST_ROS_PKG_LABUST_DIAGNOSTICS_INCLUDE_LABUST_DIAGNOSTICS_DIAGNOSTICHANDLER_H_ */
