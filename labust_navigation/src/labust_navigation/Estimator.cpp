/*
 * Estimator.cpp
 *
 *  Created on: Sep 1, 2016
 *      Author: filip
 */
#include <labust/navigation/Estimator.h>


#include <deque>
#include <stack>


using namespace labust::navigation;

template <class Model>
EstimatorContainer<Model>::EstimatorContainer():KFCore2<Model>(1,1){}

template <class Model>
EstimatorContainer<Model>::~EstimatorContainer(){}

template <class Model>
void EstimatorContainer<Model>::resetEstimator()
{

}

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
typename EstimatorContainer<Model>::measurement_vector EstimatorContainer<Model>::delayToSteps()
{

	measurement_vector tmp;
Eigen::VectorXd tmp2;

	tmp.resize(Estimator::stateNum);
	tmp.setZero();
	return tmp;//std::floor((arrival_time-measurement_time)/_ts);
}

template <class Model>
typename EstimatorContainer<Model>::state_vector EstimatorContainer<Model>::getStateVector(double delay)
{
	return;
}

template <class Model>
bool Estimator<Model>::storeCurrentData()
{
	/*** Store x, P, R data ***/
	_state.state = Estimator::getState();
	_state.p_covariance = Estimator::getStateCovariance();
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
	_past_states.push_back(_state);

	return new_delayed;
}

template <class Model>
void Estimator<Model>::estimation(FilterState state)
{
	Estimator::predict(state.input);
	bool new_arrived(false);
	for(size_t i=0; i<state.new_measurement.size(); ++i)
		if ((new_arrived = (state.new_measurement(i)>=0))) break;
	if (new_arrived)
	{
		typename Estimator::vector new_meas(Estimator::vector::Zero(state.new_measurement.size()));
		for(size_t i=0; i<state.new_measurement.size(); ++i)
				if (state.new_measurement(i)>=0)
					new_meas(i) = 1;

		Estimator::correct(Estimator::update(state.measurement, new_meas));
	}
}

template <class Model>
void Estimator<Model>::estimatorStep()
{
	/*** Mutex ***/
	boost::mutex::scoped_lock l(_filter_mux);

	/*** Store current data and check for delayed measurements ***/
	bool new_delayed = storeCurrentData();

	if(new_delayed && _enable_delay)
	{
		/*** Convert delay to discrete steps ***/
		typename Estimator::vector delay_steps = delayToSteps();
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
			typename Estimator::vector tmp_cmp;
			tmp_cmp.setConstant(Estimator::measurement_num, i);
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
		Estimator::setStateCovariance(state_p.p_covariance);
		Estimator::setState(state_p.state);

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




