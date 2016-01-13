/*********************************************************************
 *  EstimatorContainer.hpp
 *
 *  Created on: Jan 13, 2016
 *      Authors: Filip Mandic
 ********************************************************************/

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, LABUST, UNIZG-FER
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

#ifndef ESTIMATORCONTAINER_HPP_
#define ESTIMATORCONTAINER_HPP_

#include <labust/navigation/KFCore.hpp>
#include <deque>
#include <stack>

namespace labust
{
	namespace navigation
	{
		template <class Model>
		class EstimatorContainer
		{

		private:
			typedef labust::navigation::KFCore<Model> Estimator;
			typedef labust::navigation::KFCore<Model>::vector vector;
			typedef labust::navigation::KFCore<Model>::vector matrix;
			typedef labust::navigation::KFCore<Model>::measurement_vector measurement_vector;
			typedef labust::navigation::KFCore<Model>::state_vector state_vector;
			typedef labust::navigation::KFCore<Model>::input_vector input_vector;

			struct FilterState
			{
				FilterState(){}

				~FilterState(){}

				Estimator::vector input;
				Estimator::vector measurement;
				Estimator::vector new_measurement; /* Value -1 denotes no measurement, values >=0 denote measurement delay. */
				Estimator::vector state;
				Estimator::matrix p_covariance;
				Estimator::matrix r_covariance;
			};

		public:
			EstimatorContainer(){}

			~EstimatorContainer(){}

			void setMeasurementVector(measurement_vector measurement, measurement_vector new_measurement);

			void setStateVector(state_vector state);

			void setInputVector(input_vector input);

			int delayToSteps(double measurement_time, double arrival_time);

			state_vector getStateVector(double delay = 0.0);

			matrix getCovarianceMatrix(double delay = 0.0);

			FilterState getFilterState(double delay = 0.0);

			void estimatorStep();

		private:

			void storeCurrentData();

			void estimation(FilterState state);

			FilterState _state;
			std::deque<FilterState> _past_states;

			Estimator _filter;

			boost::mutex _filter_mux;

			/*** Enable delay recalculation flag ***/
			bool _enable_delay;

			double _ts;
		};
	}
}

using namespace labust::navigation;

template <class Model>
void EstimatorContainer<Model>::setMeasurementVector(measurement_vector measurement, measurement_vector new_measurement)
{
	_state.measurement = measurement;
	_state.new_measurement = new_measurement;
}

template <class Model>
void EstimatorContainer<Model>::setStateVector(state_vector state)
{
	_state.state = state;
}

template <class Model>
void EstimatorContainer<Model>::setInputVector(input_vector input)
{
	_state.input = input;
}

template <class Model>
int EstimatorContainer<Model>::delayToSteps(double measurement_time, double arrival_time)
{
	return std::floor((arrival_time-measurement_time)/_ts);
}

template <class Model>
EstimatorContainer::state_vector EstimatorContainer<Model>::getStateVector(double delay = 0.0)
{
	return;
}

template <class Model>
void EstimatorContainer<Model>::storeCurrentData()
{
	/*** Store x, P, R data ***/
	_state.state = _filter.getState();
	_state.p_covariance = _filter.getStateCovariance();
	//state.Rcov = ; // In case of time-varying measurement covariance

	/*** Check if there are delayed measurements, disable them in current step
	 *  and set delayed measurement flag ***/
	bool new_delayed(false);
	for(size_t i=0; i<_state.new_measurement.size(); ++i)
	{
		if(_state.new_measurement(i)>_ts/2)
		{
			_state.new_measurement(i) = -1;
			new_delayed = true;
		}
	}

	/*** Limit queue size ***/
	if(_past_states.size()>1000)
	{
		_past_states.pop_front();
	}
	_past_states.push_back(state);
}

template <class Model>
void EstimatorContainer<Model>::estimation(FilterState state)
{
	_filter.predict(state.input);
	bool new_arrived(false);
	for(size_t i=0; i<state.new_measurement.size(); ++i)
		if ((new_arrived = (state.new_measurement(i)>=0))) break;
	if (new_arrived)
	{
		Estimator::vector new_meas(Estimator::vector::Zero(state.new_measurement.size()));
		for(size_t i=0; i<state.new_measurement.size(); ++i)
				if (state.new_measurement(i)>=0)
					new_meas(i) = 1;

		_filter.correct(_filter.update(state.measurement, new_meas));
	}
}

template <class Model>
void EstimatorContainer<Model>::estimatorStep()
{
	/*** Mutex ***/
	boost::mutex::scoped_lock l(_filter_mux);

	/*** Store current data and check for delayed measurements ***/
	double new_delayed = storeCurrentData();

	if(new_delayed && _enable_delay)
	{
		/*** Convert delay to discrete steps ***/
		Estimator::vector delay_steps = ;
		/*** Check for maximum delay ***/
		int max_delay_steps = _state.new_measurement.maxCoeff();

		/*** If delay is bigger then buffer size assume that it is max delay ***/
		if(max_delay_steps >= _past_states.size())
			max_delay_steps = _past_states.size()-1;

		/*** Position delayed measurements in past states container
			 and store past states data in temporary stack ***/
		std::stack<FilterState> tmp_stack;
		for(size_t i=0; i<=max_delay_steps; i++)
		{
			Estimator::vector tmp_cmp;
			tmp_cmp.setConstant(Estimator::measSize, i);
			if((delay_steps.array() == tmp_cmp.array()).any() && i != 0)
			{
				FilterState tmp_state = _past_states.back();
				for(size_t j=0; j<delay_steps.size(); ++j)
				{
					if(delay_steps(j) == i)
					{
						tmp_state.new_measurement(j) = 0;
						tmp_state.measurement(j) = _state.measurement(j);
					}
				}
				tmp_stack.push(tmp_state);
			}
			else
			{
				tmp_stack.push(_past_states.back());
			}
			_past_states.pop_back();
		}

		/*** Start recalculation ***/
		/*** Load past state and covariance for max delay time instant ***/
		FilterState state_p = tmp_stack.top();
		_filter.setStateCovariance(state_p.p_covariance);
		_filter.setState(state_p.state);

		/*** Pass through stack data and recalculate filter states ***/
		while(!tmp_stack.empty()){
			state_p = tmp_stack.top();
			tmp_stack.pop();
			_past_states.push_back(state_p);

			/*** Estimation ***/
			estimation(state_p);
		}
	}else{
		/*** Estimation without delay ***/
		estimation(_state);
	}

	/*** After filter update, mark that measurements have been used ***/
	_state.new_measurement.setZero();
	/*** Unlock mutex ***/
	l.unlock();
}

#endif /* ESTIMATORCONTAINER_HPP_ */
