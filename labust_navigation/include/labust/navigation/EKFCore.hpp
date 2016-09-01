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
#ifndef EKFCORE_HPP_
#define EKFCORE_HPP_

#include <vector>
#include <cassert>

///\todo Remove this after debugging
#include <iostream>
#include <ros/ros.h>

namespace labust
{
	namespace navigation
	{
		/**
		 * This class combines models with different prediction and correction approaches.
		 */
		template <class Model>
		class EKFCore : public Model
		{
		public:

			typedef const typename Model Model;

			typedef const typename Model::matrix& matrixref;
			typedef const typename Model::vector& vectorref;
			//For MIMO systems
			typedef const typename Model::input_type& inputref;
			typedef const typename Model::output_type& outputref;

			typedef typename Model::vector vector;
			typedef typename Model::matrix matrix;
			typedef typename Model::measurement_vector measurement_vector;
			typedef typename Model::state_vector state_vector;
			typedef typename Model::input_vector input_vector;

			/**
			 * Generic constructor.
			 */
			EKFCore(){};


			/**
			 * Set the state covariance matrix value.
			 *
			 * \param P State covariance matrix.
			 */
			inline void setStateCovariance(matrixref P){this->P = P;};
			/**
			 * Get the state covariance matrix value.
			 *
			 * \return State covariance matrix.
			 */
			inline matrixref getStateCovariance(){return this->P;};
			/**
			 * Get the state covariance matrix value.
			 *
			 * \return State covariance matrix.
			 */
			inline matrixref getInovationCovariance(){return this->innovationCov;};
			/**
			 * Set the estimated state value.
			 *
			 * \param x The state value.
			 */
			inline void setState(vectorref x){this->xk_1 = x; this->x = x;};
			/**
			* Set the measurement matrices.
			*/
			void setMeasurementParameters(const typename Model::matrix& V,
				const typename Model::matrix& R)
			{
			this->V = V; this->R = R;
			}
			/**
			* Set the state matrices.
			*/
			void setStateParameters(const typename Model::matrix& W,
				const typename Model::matrix& Q)
			{
			this->W = W; this->Q = Q;
			}
			/**
			* Set the outlier coefficient.
			*/
			void setOutlierR(double outlierR)
			{
			this->outlierR = outlierR;
			}
			/**
			 * Set the state value.
			 *
			 * \return The estimated state value.
			 */
			inline vectorref getState(){return this->x;};
			/**
			 * Set the desired sampling time.
			 *
			 * \param Ts Sampling time
			 */
			inline void setTs(double Ts)
			{
				if (!Ts) throw std::invalid_argument("Cannot set zero sampling time.");
				this->Ts = Ts;
				this->initModel();
			};

			/**
			 * Get the covariance trace.
			 *
			 * \return Trace of the covariance matrix.
			 */
			inline double traceP()
			{
				return this->P.trace();
			}

			/**
			 * Perform the prediction step based on the system input.
			 *
			 * \param u Input vector.
			 */
			void predict(typename Base::inputref u = typename Model::input_type());
			/**
			 * State estimate correction based on the available measurements.
			 * Note that the user has to handle the measurement processing
			 * and populate the y_meas vector.
			 *
			 * \return Corrected state estimate.
			 */
			typename Base::vectorref correct(typename Base::outputref y_meas);

			/**
			 * Update the matrices V and H to accommodate for available measurements.
			 * Perform per element outlier rejection and correct the state with the
			 * remaining validated measurements.
			 *
			 * \return Corrected state estimate
			 */
			template <class NewMeasVector>
			typename Base::vectorref correct(
					typename Base::outputref measurements,
					NewMeasVector& newMeas,
					bool reject_outliers = true);


			/**
			 * The Kalman gain, estimate covariance and the innovation covariance matrix
			 */
			typename Model::matrix K, P, innovationCov;
			/**
			 * The innovation
			 */
			typename Model::vector innovation;
			/**
			 * Outlier rejection coefficient.
			 */
			double outlierR;

		};
	};
}
/* KFCORE_HPP_ */
#endif
