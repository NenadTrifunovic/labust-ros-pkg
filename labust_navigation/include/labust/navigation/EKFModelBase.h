/*
 * EKFModelBase.h
 *
 *  Created on: Sep 1, 2016
 *      Author: filip
 */

#ifndef LABUST_ROS_PKG_LABUST_NAVIGATION_INCLUDE_LABUST_NAVIGATION_EKFMODELBASE_H_
#define LABUST_ROS_PKG_LABUST_NAVIGATION_INCLUDE_LABUST_NAVIGATION_EKFMODELBASE_H_

#include <Eigen/Dense>
#include <boost/noncopyable.hpp>

namespace labust {
namespace navigation {
template<class precission = double>
class EKFModelBase: boost::noncopyable {
	/**
	 * This class implements a generic stochastic state space model for EKF use. When implementing models
	 * try to inherit from this class. The following naming scheme is used:
	 *
	 * x(k+1) = A*x(k) + B*u(k) + w(k)
	 * y(k) = H*x(k) + v(k)
	 *
	 * w - model noise p(w) = N(0,Q(k))
	 * v - measurement noise p(v) = N(0,R(k))
	 *
	 * In the nonlinear case:
	 *
	 * x(k+1) = f(x(k),u(k),w(k))
	 * y(k) = h(x(k),v(k))
	 *
	 * update the following jacobian matrices as:
	 * A = \frac{\partial{f}}{\partial{x}}(x(k-1),u(k),0)
	 * W = \frac{\partial{f}}{\partial{w}}(x(k-1),u(k),0)
	 * H = \frac{\partial{h}}{\partial{x}}(x(k),0)
	 * V = \frac{\partial{h}}{\partial{v}}(x(k),0)
	 *
	 * \todo Deprecate R0/R as there can be only one of them and by selecting V/V0 appropriately we
	 * can achieve the same result with one matrix less.
	 */

public:
	typedef precission numericprecission;
	typedef Eigen::Matrix<precission, Eigen::Dynamic, Eigen::Dynamic> matrix;
	typedef Eigen::Matrix<precission, Eigen::Dynamic, 1> vector;

	struct ModelParams {
		ModelParams() :
				alpha(1), beta(1), betaa(0) {
		}

		ModelParams(double alpha, double beta, double betaa) :
				alpha(alpha), beta(beta), betaa(betaa) {
		}

		inline double Beta(double val) {
			return beta + betaa * std::fabs(val);
		}

		double alpha, beta, betaa;
	};

	EKFModelBase() {
	}

	virtual ~EKFModelBase() {
	}

	virtual void processModelStep(vector x);

	virtual void measurementModelStep();

	virtual void getDerivativeAX();

	virtual void getDerivativeHX();

	/**
	 * Set the model parameters.
	 */
	void setParameters(const ModelParams& surge, const ModelParams& sway,
			const ModelParams& heave, const ModelParams& roll,
			const ModelParams& pitch, const ModelParams& yaw) {
		this->surge = surge;
		this->sway = sway;
		this->heave = heave;
		this->roll = roll;
		this->pitch = pitch;
		this->yaw = yaw;
	}

	/*** The model parameters. ***/
	ModelParams surge, sway, heave, roll, pitch, yaw;
	/***
	 * State transition,
	 * Input,
	 * Model noise covariance,
	 * Model noise transformation
	 * Measurement noise covariance,
	 * Measurement noise transformation
	 ***/
	matrix A, B, Q, W, H, R, V, H0, R0, V0;

	/*** Model sampling time ***/
	numericprecission Ts;

	/*** State and output vector. ***/
	vector xk1_, x_, y;

	/***
	 * Kalman gain,
	 * Estimate covariance,
	 * Innovation covariance matrix
	 ***/
	matrix Kgain, P, innovationCov;

	/*** The innovation ***/
	vector innovation;

	/*** Outlier rejection coefficient. ***/
	numericprecission outlierR;
};
}
}

#endif /* LABUST_ROS_PKG_LABUST_NAVIGATION_INCLUDE_LABUST_NAVIGATION_EKFMODELBASE_H_ */

