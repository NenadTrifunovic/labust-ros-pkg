/*
 * StatusHandlerBase.h
 *
 *  Created on: Jul 18, 2016
 *      Author: filip
 */

#ifndef LABUST_ROS_PKG_LABUST_DIAGNOSTICS_INCLUDE_LABUST_DIAGNOSTICS_STATUSHANDLERBASE_H_
#define LABUST_ROS_PKG_LABUST_DIAGNOSTICS_INCLUDE_LABUST_DIAGNOSTICS_STATUSHANDLERBASE_H_

#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>

namespace labust
{
	namespace diagnostic
	{
		class StatusHandlerBase
		{
		public:

			StatusHandlerBase()
			{
				this->setEntityName("No name");
				this->setEntityId("No ID");
				this->setEntityStatus(diagnostic_msgs::DiagnosticStatus::STALE);
				this->setEntityMessage("No message.");
			}

			 ~StatusHandlerBase()
			{

			}

			inline void setEntityName(const std::string& entity_name);

			inline void setEntityMessage(const std::string& entity_message);

			inline void setEntityId(const std::string& entity_id);

			inline void setEntityStatus(const signed char& status); //TODO check datatype

			void addKeyValue(const std::string& key_value_name);

			void updateKeyValue(const std::string& key_value_name, const std::string& value);

			void deleteKeyValue(const std::string& key_value_name);

			diagnostic_msgs::DiagnosticStatus entity_status_;
		};

		inline void StatusHandlerBase::setEntityName(const std::string& entity_name)
		{
			entity_status_.name.assign(entity_name);
		}

		inline void StatusHandlerBase::setEntityMessage(const std::string& entity_message)
		{
			entity_status_.message.assign(entity_message);
		}

		inline void StatusHandlerBase::setEntityId(const std::string& entity_id)
		{
			entity_status_.hardware_id.assign(entity_id);
		}

		inline void StatusHandlerBase::setEntityStatus(const signed char& status)
		{
			entity_status_.level = status;
		}

		void StatusHandlerBase::addKeyValue(const std::string& key_value_name)
		{
			ROS_INFO("Adding key value %s...", key_value_name.c_str());
			diagnostic_msgs::KeyValue new_value;
			new_value.key = key_value_name;
			new_value.value = "None";
			entity_status_.values.push_back(new_value);
			ROS_INFO("Key value %s added.", key_value_name.c_str());
		}

		void StatusHandlerBase::updateKeyValue(const std::string& key_value_name, const std::string& value)
		{
			ROS_DEBUG("Updating key value %s...", key_value_name.c_str());
			for(std::vector<diagnostic_msgs::KeyValue>::iterator it = entity_status_.values.begin();
					it != entity_status_.values.end(); ++it)
			{
				if(key_value_name.compare(it->key.c_str()) == 0)
				{
					it->value = value;
					ROS_DEBUG("Key value %s updated.", key_value_name.c_str());
					return;
				}
			}
			ROS_ERROR("No key value %s found.", key_value_name.c_str());
		}

		void StatusHandlerBase::deleteKeyValue(const std::string& key_value_name)
		{
			ROS_INFO("Deleting key value %s...", key_value_name.c_str());
			for(std::vector<diagnostic_msgs::KeyValue>::iterator it = entity_status_.values.begin();
					it != entity_status_.values.end(); ++it)
			{
				if(key_value_name.compare(it->key.c_str()) == 0)
				{
					entity_status_.values.erase(it--); //TODO Check this
					ROS_INFO("Key value %s deleted.", key_value_name.c_str());
					return;
				}
			}
			ROS_ERROR("No key value %s found.", key_value_name.c_str());
		}
	}
}

#endif /* LABUST_ROS_PKG_LABUST_DIAGNOSTICS_INCLUDE_LABUST_DIAGNOSTICS_STATUSHANDLERBASE_H_ */
