//\todo omoguci overide external event descriptiona
//\todo uvedi razlicite tipove ekternigh varijabli konitnuirane, flag, ...
/*********************************************************************
 * eventEvaluation.hpp
 *
 *  Created on: May 8, 2014
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

#ifndef EVENTEVALUATION_HPP_
#define EVENTEVALUATION_HPP_

#include <exprtk/exprtk.hpp>
#include <labust/mission/labustMission.hpp>

/*********************************************************************
 ***  EventEvaluation class definition
 ********************************************************************/

namespace labust {
	namespace event {

		class EventEvaluation{

		public:

			typedef exprtk::symbol_table<double> symbol_table_t;
			typedef exprtk::expression<double> expression_t;
			typedef exprtk::parser<double> parser_t;
			typedef exprtk::parser_error::type error_t;

			/*****************************************************************
			 ***  Class functions
			 ****************************************************************/

			EventEvaluation();

			vector<uint8_t> evaluateEvents(vector<string> events);

			double evaluateStringExpression(std::string expression_str);

			void updateSymbolTable(vector<double> stateVar, vector<double> missionVar, vector<string> missionVarNames);

			void initializeSymbolTable(vector<double> stateVar, vector<double> missionVar, vector<string> missionVarNames);

			void resetSymbolTable();

			/*********************************************************************
			 ***  Class variables
			 ********************************************************************/

		private:

			symbol_table_t symbol_table;
			boost::mutex mtx;
		};

		EventEvaluation::EventEvaluation(){

		}

		vector<uint8_t> EventEvaluation::evaluateEvents(vector<string> events){

			vector<uint8_t> eventsState;

			for(vector<string>::iterator it = events.begin(); it != events.end(); ++it){

				uint8_t tmp = (evaluateStringExpression((*it).c_str()) == 1.0)?1:0;
				eventsState.push_back(tmp);
			}

			return eventsState;
		}


		double EventEvaluation::evaluateStringExpression(std::string expression_str){

			boost::mutex::scoped_lock lock(mtx);
			expression_t expression;
			expression.register_symbol_table(symbol_table);

			parser_t parser;

			if (!parser.compile(expression_str,expression))
			{
			  ROS_ERROR("Error: %s\tExpression: %s\n",parser.error().c_str(),expression_str.c_str());

			  for (std::size_t i = 0; i < parser.error_count(); ++i)
			  {
				 error_t error = parser.get_error(i);
				 ROS_ERROR("Error: %02d Position: %02d Type: [%s] Msg: %s Expr: %s\n",
						static_cast<int>(i),
						static_cast<int>(error.token.position),
						exprtk::parser_error::to_str(error.mode).c_str(),
						error.diagnostic.c_str(),
						expression_str.c_str());
			  }
			  return -1;
			}

			return expression.value();
		}

		void EventEvaluation::initializeSymbolTable(vector<double> stateVar, vector<double> missionVar, vector<string> missionVarNames){

			symbol_table.add_constants();

			double tmp = 0.0;
			int i;

			for(i = 0; i < stateHatNum; i++ ){

				symbol_table.create_variable(stateVarNames[i]);
				symbol_table.get_variable(stateVarNames[i])->ref() = double(tmp);
			}

			i = 0;
			for(vector<double>::iterator it = missionVar.begin() ; it != missionVar.end(); ++it){

				string varName = missionVarNames[i++];

				symbol_table.create_variable(varName.c_str());
				symbol_table.get_variable(varName.c_str())->ref() = double(tmp);
			}
		}

		void EventEvaluation::updateSymbolTable(vector<double> stateVar, vector<double> missionVar, vector<string> missionVarNames){

			boost::mutex::scoped_lock lock(mtx);
			double tmp;
			int i;

			for(i = 0; i < stateHatNum; i++){

				tmp = stateVar[i];
				symbol_table.get_variable(stateVarNames[i])->ref() = double(tmp);
			}

		    i = 0;
			for(std::vector<double>::iterator it = missionVar.begin() ; it != missionVar.end(); ++it){

				string varName = missionVarNames[i++];

				double value = (*it);
				symbol_table.get_variable(varName.c_str())->ref() = double(value);
			}
		}

		void EventEvaluation::resetSymbolTable(){
			symbol_table.clear();
		}
	}
}

#endif /* EVENTEVALUATION_HPP_ */
