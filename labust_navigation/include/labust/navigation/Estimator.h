/*
 * Estimator.h
 *
 *  Created on: Sep 1, 2016
 *      Author: filip
 */

#ifndef LABUST_ROS_PKG_LABUST_NAVIGATION_INCLUDE_LABUST_NAVIGATION_ESTIMATOR_H_
#define LABUST_ROS_PKG_LABUST_NAVIGATION_INCLUDE_LABUST_NAVIGATION_ESTIMATOR_H_

namespace labust
{
	namespace navigation
	{
		template <typename Core>
		class Estimator : public Core
		{

			typedef labust::navigation::KFCore2<Model> Estimator;
			typedef typename Estimator::vector vector;
			typedef typename Estimator::matrix matrix;
			typedef typename Estimator::measurement_vector measurement_vector;
			typedef typename Estimator::state_vector state_vector;
			typedef typename Estimator::input_vector input_vector;

			struct FilterState
			{
				FilterState(){}

				~FilterState(){}

				vector input;
				vector measurement;
				vector new_measurement; /* Value -1 denotes no measurement, values >=0 denote measurement delay. */
				vector state;
				matrix p_covariance;
				matrix r_covariance;
			};


		public:
					Estimator();

					~Estimator();

					void resetEstimator();

					void setMeasurementVector(measurement_vector measurement, measurement_vector new_measurement);

					void setStateVector(state_vector state);

					void setInputVector(input_vector input);

					state_vector getStateVector(double delay = 0.0);

					matrix getCovarianceMatrix(double delay = 0.0);

					FilterState getFilterState(double delay = 0.0);

					void setSamplingTime(double ts)
					{
						Estimator::setTs(ts);
					}

					double getSamplingTime(){
						return Estimator::Ts;
					}

					void estimatorStep();

					double calculateInovationVariance();

					double caluclateConditionNumber();

				private:

					measurement_vector delayToSteps();

					bool storeCurrentData();

					void estimation(FilterState state);

					FilterState _state;
					std::deque<FilterState> _past_states;

					//Estimator _filter;

					boost::mutex _filter_mux;

					/*** Enable delay recalculation flag ***/
					bool _enable_delay;

					double _ts;

			/******** Recalculation ***************/




			/************* REjection **************/

		};
	}
}

#endif /* LABUST_ROS_PKG_LABUST_NAVIGATION_INCLUDE_LABUST_NAVIGATION_ESTIMATOR_H_ */
