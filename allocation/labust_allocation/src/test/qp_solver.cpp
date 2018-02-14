// example: construct a quadratic program from data
// the QP below is the first quadratic program example in the user manual
#include <iostream>
#include <cassert>
#include <CGAL/basic.h>
#include <CGAL/QP_models.h>
#include <CGAL/QP_functions.h>
#include <stdint.h>
// choose exact integral type
#include <CGAL/Gmpz.h>
#include <CGAL/MP_Float.h>
typedef CGAL::Gmpzf ET;
//typedef CGAL::MP_double ET;
#include <ros/ros.h>
#include <Eigen/Dense>
#include <labust/math/NumberManipulation.hpp>
// program and solution types
typedef CGAL::Quadratic_program<double> Program;
typedef CGAL::Quadratic_program_solution<ET> Solution;

//typedef CGAL::Simple_cartesian<double> K;
//typedef K::Point_2                     Point;
//typedef K::Segment_2                   Segment;

struct XConfiguration
{
	enum{Xmin=0, Ymin=0, Nmin=0, Xmax=1, Ymax=1, Nmax=1};
	enum{t1=0,t2,t3,t4};

	///TODO: Generalize for any B
	void coerce2poly(const Eigen::Vector3d& tau, double maxN, Eigen::Vector3d& tau_ach)
	{
		//Limit N to maximum torque
		Eigen::Vector3d tau_des(tau);
		tau_des(2) = labust::math::coerce(tau(2), maxN*Nlim(Nmin), maxN*Nlim(Nmax));

		Eigen::MatrixXd polytop;
		calculateBounds(tau_des(2),polytop);

		Eigen::Vector2d tauxy;
		tauxy<<tau(0),tau(1);

		if (!insidePoly(tauxy, polytop))
		{
			//ROS_WARN("Not inside polytop.");
			Eigen::Matrix2d line;
			line<<0,0,tau_des(0),tau_des(1);
			Eigen::MatrixXd poly2D = polytop.block(0,0,9,2);
			Eigen::Vector2d icp;
			if (!intersectPoly(line, polytop, icp))
			{
				ROS_ERROR("Empty intersect.");
				tau_des(0) = 0;
				tau_des(1) = 0;
			}
			else
			{
				tau_des(0) = 0.999*icp(0);
				tau_des(1) = 0.999*icp(1);
			}

			//ROS_INFO("Remapped from: (%f,%f,%f) -> (%f,%f,%f)",
			//		tau(0), tau(1), tau(2),
			//		tau_des(0), tau_des(1), tau_des(2));
		}
		else
		{
			//ROS_INFO("Inside poly.");
		}

		tau_ach = tau_des;
	}

	bool insidePoly(const Eigen::Vector2d& point, const Eigen::MatrixXd& poly)
	{
		int j = poly.rows()-1;
		double x = point(0);
		double y = point(1);

		bool flag = false;

		for(int i=0; i<poly.rows(); ++i)
		{
			double vertxi = poly(i,0);
			double vertyi = poly(i,1);
			double vertxj = poly(j,0);
			double vertyj = poly(j,1);

			if (((vertyi > y) != (vertyj > y)) &&
					(x < (vertxj-vertxi)*(y-vertyi)/(vertyj-vertyi) + vertxi)) flag = !flag;
			j=i;
		}

		return flag;
	}

	///TODO: Generalize for any matrix B, one generic helper function for all three limits
	void calculateBounds(double N, Eigen::MatrixXd& polytop)
	{
		//ROS_INFO("Calculate bounds.");
		double F24min = fmin(t2) - fmax(t4);
		double F24max = fmax(t2) - fmin(t4);

		double F34min = fmin(t3) - fmax(t4);
		double F34max = fmax(t3) - fmin(t4);

		Eigen::MatrixXd XminIncp, XmaxIncp, YminIncp, YmaxIncp;
		double Xmin = calculateBoundsXmin(N, F24min, XminIncp);
		double Xmax = calculateBoundsXmax(N, F24max, XmaxIncp);
		double Ymin = calculateBoundsYmin(N, F34max, YminIncp);
		double Ymax = calculateBoundsYmax(N, F34min, YmaxIncp);

		//std::cout<<XminIncp<<std::endl;
		//std::cout<<XmaxIncp<<std::endl;
		//std::cout<<YminIncp<<std::endl;
		//std::cout<<YmaxIncp<<std::endl;
		polytop.resize(9,3);
		polytop << Xmin, XminIncp.row(1).minCoeff(), N,
				Xmin, XminIncp.row(1).maxCoeff(), N,
				YmaxIncp.row(0).minCoeff(), Ymax, N,
				YmaxIncp.row(0).maxCoeff(), Ymax, N,
				Xmax, XmaxIncp.row(1).maxCoeff(), N,
				Xmax, XmaxIncp.row(1).minCoeff(), N,
				YminIncp.row(0).maxCoeff(), Ymin, N,
				YminIncp.row(0).minCoeff(), Ymin, N,
				Xmin, XminIncp.row(1).minCoeff(), N;
	}

	double calculateBoundsXmin(double N, double F24min, Eigen::MatrixXd& XminIncp)
	{
		// Analyze thruster pairs
		double F13 = N - F24min;
		double F13max = fmax(t3) - fmin(t1);
		double F13min = fmin(t3) - fmax(t1);
		// If outside achievable move the pair on the achivable edge
		if (F13 < F13min)
			F13 = F13min;
		else if (F13 > F13max)
			F13 = F13max;

		// Recalculate the second thruster pair
		double F24 = N-F13;
		double Xmin = (-N + 2*F24)*sqrt(2)/2;

		// Find intersections
		Eigen::MatrixXd icp13, icp24;
		// f3 = f1 + F13
		intersectRect(F13, fmin(t1), fmax(t1), fmin(t3), fmax(t3), icp13);
		// f2 = f4 + F24
		intersectRect(F24, fmin(t4), fmax(t4), fmin(t2), fmax(t2), icp24);

		// Check if something fishy is going on
		if ((icp13.cols() > 2) || (icp13.cols() < 1))
			ROS_ERROR("Intersection error: F13 for N=%f, %ld", N, icp13.cols());
		if ((icp24.cols() > 2) || (icp24.cols() < 1))
			ROS_ERROR("Intersection error: F24 for N=%f, %ld", N, icp24.cols());

		for(int i=0; i < icp13.cols(); ++i)
		{
			double f1 = icp13(0,i);
			double f3 = icp13(1,i);
			for(int j=0; j < icp24.cols(); ++j)
			{
				double f2 = icp24(1,j);
				double f4 = icp24(0,j);
				double Xicp = ( f1 + f2 - f3 - f4)*sqrt(2)/2;
				double Yicp = (-f1 + f2 - f3 + f4)*sqrt(2)/2;
				double Nicp = (-f1 + f2 + f3 - f4);

				XminIncp.conservativeResize(3, XminIncp.cols()+1);
				XminIncp.col(XminIncp.cols()-1) << Xicp, Yicp, Nicp;
			}
		}

		return Xmin;
	}

	double calculateBoundsXmax(double N, double F24max, Eigen::MatrixXd& XmaxIncp)
	{
		// Analyze thruster pairs
		double F13 = N - F24max;
		double F13max = fmax(t3) - fmin(t1);
		double F13min = fmin(t3) - fmax(t1);
		// If outside achievable move the pair on the achivable edge
		if (F13 < F13min)
			F13 = F13min;
		else if (F13 > F13max)
			F13 = F13max;

		// Recalculate the second thruster pair
		double F24 = N-F13;
		double Xmax = (-N + 2*F24)*sqrt(2)/2;

		// Find intersections
		Eigen::MatrixXd icp13, icp24;
		// f3 = f1 + F13
		intersectRect(F13, fmin(t1), fmax(t1), fmin(t3), fmax(t3), icp13);
		// f2 = f4 + F24
		intersectRect(F24, fmin(t4), fmax(t4), fmin(t2), fmax(t2), icp24);
		// Check if something fishy is going on

		if ((icp13.cols() > 2) || (icp13.cols() < 1))
			ROS_ERROR("Intersection error: F13 for N=%f, %ld", N, icp13.cols());
		if ((icp24.cols() > 2) || (icp24.cols() < 1))
			ROS_ERROR("Intersection error: F24 for N=%f, %ld", N, icp24.cols());

		for(int i=0; i < icp13.cols(); ++i)
		{
			double f1 = icp13(0,i);
			double f3 = icp13(1,i);
			for(int j=0; j < icp24.cols(); ++j)
			{
				double f2 = icp24(1,j);
				double f4 = icp24(0,j);
				double Xicp = ( f1 + f2 - f3 - f4)*sqrt(2)/2;
				double Yicp = (-f1 + f2 - f3 + f4)*sqrt(2)/2;
				double Nicp = (-f1 + f2 + f3 - f4);
				XmaxIncp.conservativeResize(3, XmaxIncp.cols()+1);
				XmaxIncp.col(XmaxIncp.cols()-1) << Xicp, Yicp, Nicp;
			}
		}

		return Xmax;
	}

	double calculateBoundsYmin(double N, double F34max, Eigen::MatrixXd& YminIncp)
	{
		double F12 = N - F34max;
		double F12max = fmax(t2) - fmin(t1);
		double F12min = fmin(t2) - fmax(t1);
		// If outside achievable move the pair on the achivable edge
		if (F12 < F12min)
			F12 = F12min;
		else if (F12 > F12max)
			F12 = F12max;

		// Recalculate the second thruster pair
		double F34 = N-F12;
		double Ymin = (N - 2*F34)*sqrt(2)/2;

		// Find intersections
		Eigen::MatrixXd icp12, icp34;
		// f2 = f1 + F12
		intersectRect(F12, fmin(t1), fmax(t1), fmin(t2), fmax(t2), icp12);
		// f3 = f4 + F34
		intersectRect(F34, fmin(t4), fmax(t4), fmin(t3), fmax(t3), icp34);
		// Check if something fishy is going on

		if ((icp12.cols() > 2) || (icp12.cols() < 1))
			ROS_ERROR("Intersection error: F12 for N=%f, %ld", N, icp12.cols());
		if ((icp34.cols() > 2) || (icp34.cols() < 1))
			ROS_ERROR("Intersection error: F34 for N=%f, %ld", N, icp34.cols());

		for(int i=0; i < icp12.cols(); ++i)
		{
			double f1 = icp12(0,i);
			double f2 = icp12(1,i);
			for(int j=0; j < icp34.cols(); ++j)
			{
				double f3 = icp34(1,j);
				double f4 = icp34(0,j);
				double Xicp = ( f1 + f2 - f3 - f4)*sqrt(2)/2;
				double Yicp = (-f1 + f2 - f3 + f4)*sqrt(2)/2;
				double Nicp = (-f1 + f2 + f3 - f4);
				YminIncp.conservativeResize(3, YminIncp.cols()+1);
				YminIncp.col(YminIncp.cols()-1) << Xicp, Yicp, Nicp;
			}
		}

		return Ymin;
	}

	double calculateBoundsYmax(double N, double F34min, Eigen::MatrixXd& YmaxIncp)
	{
		double F12 = N - F34min;
		double F12max = fmax(t2) - fmin(t1);
		double F12min = fmin(t2) - fmax(t1);
		// If outside achievable move the pair on the achivable edge
		if (F12 < F12min)
			F12 = F12min;
		else if (F12 > F12max)
			F12 = F12max;

		// Recalculate the second thruster pair
		double F34 = N-F12;
		double Ymax = (N - 2*F34)*sqrt(2)/2;

		// Find intersections
		Eigen::MatrixXd icp12, icp34;
		// f2 = f1 + F12
		intersectRect(F12, fmin(t1), fmax(t1), fmin(t2), fmax(t2), icp12);
		// f3 = f4 + F34
		intersectRect(F34, fmin(t4), fmax(t4), fmin(t3), fmax(t3), icp34);
		// Check if something fishy is going on

		if ((icp12.cols() > 2) || (icp12.cols() < 1))
			ROS_ERROR("Intersection error: F12 for N=%f, got: %ld", N, icp12.cols());
		if ((icp34.cols() > 2) || (icp34.cols() < 1))
			ROS_ERROR("Intersection error: F34 for N=%f, got: %ld", N, icp34.cols());

		for(int i=0; i < icp12.cols(); ++i)
		{
			double f1 = icp12(0,i);
			double f2 = icp12(1,i);
			for(int j=0; j < icp34.cols(); ++j)
			{
				double f3 = icp34(1,j);
				double f4 = icp34(0,j);
				double Xicp = ( f1 + f2 - f3 - f4)*sqrt(2)/2;
				double Yicp = (-f1 + f2 - f3 + f4)*sqrt(2)/2;
				double Nicp = (-f1 + f2 + f3 - f4);
				YmaxIncp.conservativeResize(3, YmaxIncp.cols()+1);
				YmaxIncp.col(YmaxIncp.cols()-1) << Xicp, Yicp, Nicp;
			}
		}

		return Ymax;
	}

	void intersectRect(double b, double xmin,
			double xmax, double ymin, double ymax,
			Eigen::MatrixXd& icp)
	{
		//ROS_INFO("Intersect Rect.");
		//Search for interceptions of y = x + b and rectangle
		// xmax
		double y = xmax + b;
		double tol = 0.00001;
		if ((ymin - y <= tol) && (y - ymax <= tol))
		{
			icp.conservativeResize(2, icp.cols() + 1);
			icp.col(icp.cols()-1) << xmax, y;
		}
		// xmin
		y = xmin + b;
		if ((ymin - y <= tol ) && (y - ymax <= tol))
		{
			icp.conservativeResize(2, icp.cols() + 1);
			icp.col(icp.cols()-1) << xmin, y;
		}
		// ymax
		double x = ymax - b;
		if ((xmin - x <= tol) && (x - xmax <= tol))
		{
			icp.conservativeResize(2, icp.cols() + 1);
			icp.col(icp.cols()-1) << x, ymax;
		}
		// ymin
		x = ymin - b;
		if ((xmin - x <= tol) && (x -xmax <= tol))
		{
			icp.conservativeResize(2, icp.cols() + 1);
			icp.col(icp.cols()-1)  << x, ymin;
		}
	}

	bool intersectLine(const Eigen::Matrix2d& line1, const Eigen::Matrix2d& line2,
			Eigen::Vector2d& icp)
	{
		bool found = false;
		double tol = 0.0001;

		double x1 = line1(0,0);
		double x2 = line1(1,0);
		double x3 = line2(0,0);
		double x4 = line2(1,0);
		double y1 = line1(0,1);
		double y2 = line1(1,1);
		double y3 = line2(0,1);
		double y4 = line2(1,1);

		//Skip if parallel
		//ROS_INFO("x1-y1: %f %f",x1,y1);
		//ROS_INFO("x2-y2: %f %f",x2,y2);
		//ROS_INFO("x3-y3: %f %f",x3,y3);
		//ROS_INFO("x4-y4: %f %f",x4,y4);
		double den = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4);
		if (den != 0)
		{
			double nom1 = (x1*y2-y1*x2)*(x3-x4) - (x1-x2)*(x3*y4-y3*x4);
			double nom2 = (x1*y2-y1*x2)*(y3-y4) - (y1-y2)*(x3*y4-y3*x4);
			double x = nom1/den;
			double y = nom2/den;
			// Test if on line segment
			bool tx1 = (std::min(x1,x2) - x < tol) && (x - std::max(x1,x2) < tol);
			bool ty1 = (std::min(y1,y2) - y < tol) && (y - std::max(y1,y2) < tol);
			bool tx2 = (std::min(x3,x4) - x < tol) && (x - std::max(x3,x4) < tol);
			bool ty2 = (std::min(y3,y4) - y < tol) && (y - std::max(y3,y4) < tol);
			found = (tx1 && ty1 && tx2 && ty2);

			if (found)
			{
				//ROS_INFO("Found intercept: %f %f",x,y);
				icp << x,y;
			}
		}
		//else
		//ROS_INFO("Parallel lines");

		return found;
	}

	///TODO: add multisoultion, etc.
	bool intersectPoly(const Eigen::Matrix2d& line, const Eigen::MatrixXd& poly, Eigen::Vector2d& icp)
	{
		bool found = false;
		int j = poly.rows()-1;
		for(int i=0; i<poly.rows(); ++i)
		{
			Eigen::Matrix2d line2;
			line2 << poly(j,0), poly(j,1), poly(i,0), poly(i,1);
			if ((found = intersectLine(line, line2, icp))) break;
			j = i;
		}
		return found;
	}

	///TODO: Generalize for any matrix B
	void updateLimits(const Eigen::Vector4d& fmin, const Eigen::Vector4d& fmax)
	{
		// X = (f1 + f2 - f3 - f4)*sqrt(2)/2
		Xlim(Xmin) = (fmin(t1) + fmin(t2) - fmax(t3) - fmax(t4))*sqrt(2)/2;
		Xlim(Xmax) = (fmax(t1) + fmax(t2) - fmin(t3) - fmin(t4))*sqrt(2)/2;
		// Y = (-f1 + f2 - f3 + f4)*sqrt(2)/2
		Ylim(Ymin) = (-fmax(t1) + fmin(t2) - fmax(t3) + fmin(t4))*sqrt(2)/2;
		Ylim(Ymax) = (-fmin(t1) + fmax(t2) - fmin(t3) + fmax(t4))*sqrt(2)/2;
		// N = -f1 + f2 + f3 - f4
		Nlim(Nmin) = -fmax(t1) + fmin(t2) + fmin(t3) - fmax(t4);
		Nlim(Nmax) = -fmin(t1) + fmax(t2) + fmax(t3) - fmin(t4);

		this->fmin = fmin;
		this->fmax = fmax;
	}

	//Limits on X
	Eigen::Vector2d Xlim;
	//Limits on Y
	Eigen::Vector2d Ylim;
	//Limits on N
	Eigen::Vector2d Nlim;
	//Maximums for each thruster
	Eigen::Vector4d fmax;
	//Minimums for each thruster
	Eigen::Vector4d fmin;
};


bool saturate(Eigen::Vector4d& t,
		const Eigen::Vector4d& pmin, const Eigen::Vector4d& pmax)
{
	double scalef(1.0), scale(0.0);
	for (int i=0;i<t.size(); ++i)
	{
		if (t(i) >= 0)
			scale = t(i)/pmax(i);
		else
			scale = t(i)/pmin(i);

		if (scale > scalef) scalef=scale;
	}
	t = t/scalef;

	return (scalef > 1.0);
}

double get_rand(double max)
{
	const int mux = 1000000;
	return (2*double(rand())/RAND_MAX-1.0)*max;
}

int main(int argc, char* argv[]) {
	ros::init(argc,argv, "qp_solver_test");
	ros::NodeHandle nh;
	ros::Time t0 = ros::Time::now();
	int ap=10000;
	// by default, we have a nonnegative QP with Ax == b
	Program qp (CGAL::EQUAL, true, -ap, true, ap);

	// now set the non-default entries:
	const int f1 = 0;
	const int f2 = 1;
	const int f3 = 2;
	const int f4 = 3;
	const int X = 0;
	const int Y = 1;
	const int N = 2;

	double max_time=0;
	int iter=0;

	double Xamp = 2;
	double Yamp = 2;
	double Namp = 0;
	double Xstep = 0.01;
	double Ystep = 0.01;
	double Nstep = 0.1;

	std::ofstream taur("taur.csv");
	std::ofstream taua("taua.csv");

	std::cout<<RAND_MAX<<std::endl;
	std::cout<<rand()<<std::endl;
	std::cout<<get_rand(Namp)<<std::endl;

	Eigen::Vector4d fmax, fmin;
	XConfiguration x;
	double avg_time=0;
	ros::Time tstart = ros::Time::now();
	for (double tauX=-Xamp; tauX<=Xamp; tauX+=Xstep)
		for (double tauY=-Yamp; tauY<=Yamp; tauY+=Ystep)
			for (double tauN=-Namp; tauN<=Namp; tauN+=Nstep)
			{
				if (!ros::ok()) break;
				ros::Time t0 = ros::Time::now();
				//double tau[]={tauX,tauY,tauN};
				double tau[]={get_rand(Xamp),get_rand(Yamp),get_rand(Namp)};
				fmax<<0.9855, 0.9844, 0.9979, 1.0000;
				fmin<<-0.1283, -0.1283, -0.4104, -0.4104;
				x.updateLimits(fmin, fmax);
				Eigen::Vector3d taud;
			  taud<<tau[X], tau[Y], tau[N];
				//taud<<2.0, 0.0, 0.0;
				Eigen::Vector3d tau_ach;
				x.coerce2poly(taud, 0.5, tau_ach);
				tau[X] = tau_ach(X);
				tau[Y] = tau_ach(Y);
				tau[N] = tau_ach(N);

				/*Eigen::Matrix2d line1, line2;
				Eigen::Vector2d icp;
				line1<<0,0,0,10;
				line2<<-5,5,5,5;
				if (x.intersectLine(line1, line2, icp))
				{
					ROS_INFO("line intercepts at: (%f, %f)", icp(0), icp(1));
				}
				else
				{
					ROS_ERROR("No intersection.");
				}*/

				//int tau[]={0.6*ap,1.0*ap,0.0*ap};

				double c = cos(M_PI/4);
				double l = 1;
				//int c = cos(M_PI/4)*ap;
				//int l = 1*ap;
				//A
				qp.set_a(f1, X,  c); qp.set_a(f2, X, c);qp.set_a(f3, X, -c); qp.set_a(f4, X, -c);
				qp.set_a(f1, Y, -c); qp.set_a(f2, Y, c);qp.set_a(f3, Y, -c); qp.set_a(f4, Y, c);
				qp.set_a(f1, N, -l); qp.set_a(f2, N, l);qp.set_a(f3, N, l); qp.set_a(f4, N, -l);
				//b
				qp.set_b(X, tau[X]); qp.set_b(Y, tau[Y]); qp.set_b(N, tau[N]);

				//int fmax[]={0.9855*ap, 0.9844*ap, 0.9979*ap, 1.0000*ap};
				//int fmin[]={-0.1283*ap, -0.1283*ap, -0.4104*ap, -0.4104*ap};

				qp.set_u(f1, true, fmax(f1));qp.set_l(f1, true, fmin(f1));
				qp.set_u(f2, true, fmax(f2));qp.set_l(f2, true, fmin(f2));
				qp.set_u(f3, true, fmax(f3));qp.set_l(f3, true, fmin(f3));
				qp.set_u(f4, true, fmax(f4));qp.set_l(f4, true, fmin(f4));

				qp.set_d(f1, f1, 2);
				qp.set_d(f2, f2, 2);
				qp.set_d(f3, f3, 2);
				qp.set_d(f4, f4, 2);

				// solve the program, using ET as the exact type
				Solution s = CGAL::solve_quadratic_program(qp, ET());
				assert (s.solves_quadratic_program(qp));
				//ROS_INFO("%f", tauX);
				/*if (fabs(tauX - 0.0) < 0.0001)
		{
			ROS_INFO("Ola");
			ros::Rate(10).sleep();
		}*/
				++iter;
				double dt = (ros::Time::now()-t0).toSec();
				if (dt > max_time) max_time=dt;
				taur<<tauX<<","<<tauY<<","<<tauN<<std::endl;
				if (!s.is_infeasible())
				{
					Solution::Variable_value_iterator fs = s.variable_values_begin();
					//std::cout<<"Solution:";
					Eigen::Vector4d f;
					int i = 0;
					while (fs != s.variable_values_end())
					{
						f[i++] = CGAL::to_double(*fs++);
					}

					if (saturate(f, fmin, fmax))
					{
						std::cout<<"Contract breach!"<<std::endl;
					}

					Eigen::Matrix<double,3,4> B;
					float cp(cos(M_PI/4)),sp(sin(M_PI/4));
					B<<cp,cp,-cp,-cp,
							-sp,sp,-sp,sp,
							-1,1,1,-1;
					Eigen::Vector3d ta = B*f;
					taua << ta(0) << "," << ta(1) << "," << ta(2) << std::endl;

					double dx = fabs(tau[X] - ta(X));
					double dy = fabs(tau[Y] - ta(Y));
					double dz = fabs(tau[N] - ta(N));
					double tol = 0.0001;
					if ((dx > tol) || (dy > tol) || (dz > tol))
					{
						ROS_ERROR("Contract breached. Expected: (%f, %f, %f) "
								"and got: (%f, %f, %f)",
								tau[X], tau[Y], tau[N], ta(X), ta(Y), ta(N));
					}
				}
				else
				{
					ROS_ERROR("Infeasible solution.");
				}
			}

	//Xconfiguration testing
//	ROS_INFO("Xconfiguration testing.");
//	XConfiguration x;
//	x.updateLimits(fmin, fmax);
//	Eigen::MatrixXd polytop;
//	x.calculateBounds(0,polytop);
//	std::cout<<polytop<<std::endl;


	// output solution
	//std::cout << s <<std::endl;
	//Solution::Variable_value_iterator fs = s.variable_values_begin();
	//std::cout<<"Solution:";
	//while (fs != s.variable_values_end())
	//{
	//	std::cout<<CGAL::to_double(*fs++)<<" ";
	//}
	//std::cout<<std::endl;
	std::cout <<"Max iteration time:" << max_time << ", iter:" << iter <<std::endl;
	std::cout <<"Total time:" << (ros::Time::now() - tstart).toSec() << std::endl;
	return 0;
}

