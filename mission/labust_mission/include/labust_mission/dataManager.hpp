/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014-2018, LABUST, UNIZG-FER
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
*  "AS IS" AND ANExternalEventY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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

#ifndef DATAMANAGER_HPP_
#define DATAMANAGER_HPP_

#include <labust/mission/labustMission.hpp>

/*********************************************************************
 ***  DataManager class definition
 ********************************************************************/

namespace labust {
	namespace data {

		class DataManager{

		public:

			/*****************************************************************
			 ***  Class functions
			 ****************************************************************/

			DataManager();

			void updateStateVar(const auv_msgs::NavigationStatus::ConstPtr& data);

			void updateMissionVar(int missionVarID, double value);

			void setMissionVar(double value);

			void setMissionVarNames(string name);

			void updateEventsVar(vector<uint8_t> values);

			vector<double> getStateVar();

			vector<double> getMissionVar();

			vector<string> getMissionVarNames();

			vector<uint8_t> getEventsVar();

			void reset();


			/*********************************************************************
			 ***  Class variables
			 ********************************************************************/

		private:

			/* State Hat variables */
			vector<double> stateHatVar;

			/* Events data variables */
			vector<uint8_t> eventsVar;

			/* Mission specific variables */
			vector<double> missionVar;
			vector<string> missionVarNames;
		};

		DataManager::DataManager(){

			stateHatVar.resize(stateHatNum);
			eventsVar.clear();
			missionVar.clear();
			missionVarNames.clear();

		}

		void DataManager::updateStateVar(const auv_msgs::NavigationStatus::ConstPtr& data){

			stateHatVar[u] = data->body_velocity.x;
			stateHatVar[v] = data->body_velocity.y;
			stateHatVar[w] = data->body_velocity.z;
			stateHatVar[r] = data->orientation_rate.z;

			stateHatVar[x] = data->position.north;
			stateHatVar[y] = data->position.east;
			stateHatVar[z] = data->position.depth;
			stateHatVar[psi] = data->orientation.z;

			stateHatVar[x_var] = data->position_variance.north;
			stateHatVar[y_var] = data->position_variance.east;
			stateHatVar[z_var] = data->position_variance.depth;
			stateHatVar[psi_var] = data->orientation_variance.z;

			stateHatVar[alt] = data->altitude;

		}

		void DataManager::updateMissionVar(int missionVarID, double value){

			missionVar.at(missionVarID-1) = value;
			/* OVDJE DODAJ KAKO DA HENDLA EXCEPTION */
		}

		void DataManager::setMissionVar(double value){

				missionVar.push_back(value);
		}

		void DataManager::setMissionVarNames(string name){

			missionVarNames.push_back(name.c_str());
			/* OVDJE DODAJ KAKO DA HENDLA EXCEPTION */
		}

		void DataManager::updateEventsVar(vector<uint8_t> values){

			eventsVar = values;
			/* OVDJE DODAJ KAKO DA HENDLA EXCEPTION */
		}

		vector<double> DataManager::getStateVar(){
			return stateHatVar;
		}

		vector<double> DataManager::getMissionVar(){
			return missionVar;
		}

		vector<uint8_t> DataManager::getEventsVar(){
			return eventsVar;
		}

		vector<string> DataManager::getMissionVarNames(){
			return missionVarNames;
		}

		void DataManager::reset(){

			missionVar.clear();
			missionVarNames.clear();
			eventsVar.clear();
		}
	}
}

#endif /* DATAMANAGER_HPP_ */
