/*
 * ekf_3d_node.cpp
 *
 *  Created on: Sep 1, 2016
 *      Author: filip
 */

#include <labust/navigation/EKFModelBase.h>
#include <labust/navigation/Estimator.h>
#include <labust/simulation/DynamicsParams.hpp>


#include <ros/ros.h>

namespace labust
{
	namespace navigation
	{
		class TestModel : public EKFModelBase<>
		{
			typedef const EKFModelBase<> ModelBase;
			enum {u=0,v,w,p,q,r,xp,yp,zp,phi,theta,psi,xc,yc,b,buoyancy,roll_restore,pitch_restore,altitude, stateNum};
			//enum {stateNum = 12};
			enum {inputSize = 6};
			enum {measSize = 7};
			//\todo See how to deal with K_roll
			enum {X=0,Y,Z,K,M,N};

			//template <class precission>
			void processModelStep(ModelBase::vector& x, const ModelBase::vector& input)
			{
				x(u) += Ts*(-surge.Beta(x(u))/surge.alpha*x(u) + 1/surge.alpha * input(X));
				double vd = -sway.Beta(x(v))/sway.alpha*x(v) + 1/sway.alpha * input(Y);
				x(v) += Ts*(-sway.Beta(x(v))/sway.alpha*x(v) + 1/sway.alpha * input(Y));
				x(w) += Ts*(-heave.Beta(x(w))/heave.alpha*x(w) + 1/heave.alpha * (input(Z) + x(buoyancy)));
				//x(p) += Ts*(-roll.Beta(x(p))/roll.alpha*x(p) + 1/roll.alpha * (input(Kroll) + x(roll_restore)));
				x(q) += Ts*(-pitch.Beta(x(p))/pitch.alpha*x(q) + 1/pitch.alpha * (input(M) + x(pitch_restore)));
				//double acc = (x(v)>0)?acc_starboard:acc_port;
				//double vec = (x(v)>0)?vec_starboard:vec_port;
				//x(r) += Ts*(-yaw.Beta(x(r))/yaw.alpha*x(r) + 1/yaw.alpha * input(N) + 0*x(b) - use_sc*vec*x(v) - acc*use_sc*vd);

				double xdot = x(u)*cos(x(psi)) - x(v)*sin(x(psi)) + x(xc);
				double ydot = x(u)*sin(x(psi)) + x(v)*cos(x(psi)) + x(yc);
				x(xp) += Ts * xdot;
				x(yp) += Ts * ydot;
				x(zp) += Ts * x(w);
				x(altitude) += -Ts * x(w);
				//\todo This is not actually true since angles depend on each other
				//\todo Also x,y are dependent on the whole rotation matrix.
				//\todo We make a simplification here for testing with small angles ~10°
				//x(phi) += Ts * x(p);
				x(theta) += Ts * x(q);
				x(psi) += Ts * x(r);

			}

			void measurementModelStep(const ModelBase::vector& x, ModelBase::vector& ynl);

			void getDerivativeAWX(const ModelBase::vector& x, const ModelBase::vector& input, ModelBase::matrix& A, ModelBase::matrix& W)
			{
				A = matrix::Identity(stateNum, stateNum);

				A(u,u) = 1-Ts*(surge.beta + 2*surge.betaa*fabs(x(u)))/surge.alpha;
				A(v,v) = 1-Ts*(sway.beta + 2*sway.betaa*fabs(x(v)))/sway.alpha;
				A(w,w) = 1-Ts*(heave.beta + 2*heave.betaa*fabs(x(w)))/heave.alpha;
				A(w,buoyancy) = Ts/heave.alpha;
				//A(p,p) = 1-Ts*(roll.beta + 2*roll.betaa*fabs(x(p)))/roll.alpha;
				//A(p,roll_restore) = Ts/roll.alpha;
				A(q,q) = 1-Ts*(pitch.beta + 2*pitch.betaa*fabs(x(q)))/pitch.alpha;
				A(q,pitch_restore) = Ts/pitch.alpha;
				A(r,r) = 1-Ts*(yaw.beta + 2*yaw.betaa*fabs(x(r)))/yaw.alpha;
				//A(r,v) = Ts*kvr; // THIS WAS COMMENTED OUT ON 29.06.2015.
				//A(r,b) = Ts;

				A(xp,u) = Ts*cos(x(psi));
				A(xp,v) = -Ts*sin(x(psi));
				A(xp,psi) = Ts*(-x(u)*sin(x(psi)) - x(v)*cos(x(psi)));
				A(xp,xc) = Ts;

				A(yp,u) = Ts*sin(x(psi));
				A(yp,v) = Ts*cos(x(psi));
				A(yp,psi) = Ts*(x(u)*cos(x(psi)) - x(v)*sin(x(psi)));
				A(yp,yc) = Ts;

				A(zp,w) = Ts;
				//	//\todo If you don't want the altitude to contribute comment this out.
				A(altitude,w) = -Ts;

				//	A(phi,p) = Ts;
				A(theta,q) = Ts;
				A(psi,r) = Ts;
			}

			void getDerivativeHX(const ModelBase::vector& x, const ModelBase::vector& input, ModelBase::matrix& Hnl){
//				Hnl=matrix::Identity(stateNum,stateNum);
//				ynl = Hnl*x;
//
//				switch (dvlModel)
//				{
//				case 1:
//					//Correct the nonlinear part
//					ynl(u) = x(u)+x(xc)*cos(x(psi))+x(yc)*sin(x(psi));
//					ynl(v) = x(v)-x(xc)*sin(x(psi))+x(yc)*cos(x(psi));
//
//					//Correct for the nonlinear parts
//					Hnl(u,u) = 1;
//					Hnl(u,xc) = cos(x(psi));
//					Hnl(u,yc) = sin(x(psi));
//					Hnl(u,psi) = -x(xc)*sin(x(psi)) + x(yc)*cos(x(psi));
//
//					Hnl(v,v) = 1;
//					Hnl(v,xc) = -sin(x(psi));
//					Hnl(v,yc) = cos(x(psi));
//					Hnl(v,psi) = -x(xc)*cos(x(psi)) - x(yc)*sin(x(psi));
//					break;
//				case 2:
//					//Correct the nonlinear part
//					y(u) = x(u)*cos(x(psi)) - x(v)*sin(x(psi)) + x(xc);
//					y(v) = x(u)*sin(x(psi)) + x(v)*cos(x(psi)) + x(yc);
//
//					//Correct for the nonlinear parts
//					Hnl(u,xc) = 1;
//					Hnl(u,u) = cos(x(psi));
//					Hnl(u,v) = -sin(x(psi));
//					Hnl(u,psi) = -x(u)*sin(x(psi)) - x(v)*cos(x(psi));
//
//					Hnl(v,yc) = 1;
//					Hnl(v,u) = sin(x(psi));
//					Hnl(v,v) = cos(x(psi));
//					Hnl(v,psi) = x(u)*cos(x(psi)) - x(v)*sin(x(psi));
//					break;
//				}
			}
		};

		class EKF3DNode
		{
		public:
			void start();
		};
	}
}


using namespace labust::navigation;

/*********************************************************************
 *** Main loop
 ********************************************************************/

void EKF3DNode::start()
{
  ros::NodeHandle ph("~");
  double Ts(0.1);
  ph.param("Ts", Ts, Ts);
  ros::Rate rate(1 / Ts);
  //nav.setTs(Ts);

  /*** Initialize time (for delay calculation) ***/
  //currentTime = ros::Time::now().toSec();

  while (ros::ok())
  {

  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "ekf_3d_node");
  EKF3DNode nav;
  nav.start();
  return 0;
}
