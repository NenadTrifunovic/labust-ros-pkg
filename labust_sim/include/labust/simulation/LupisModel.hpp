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
*
*  Author: Dula Nad
*  Created: 01.02.2010.
*********************************************************************/


//***************************************************************************
// Copyright 2007-2016 Universidade do Porto - Faculdade de Engenharia      *
// Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                  *
//***************************************************************************
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Universidade do Porto. For licensing   *
// terms, conditions, and further information contact lsts@fe.up.pt.        *
//                                                                          *
// European Union Public Licence - EUPL v.1.1 Usage                         *
// Alternatively, this file may be used under the terms of the EUPL,        *
// Version 1.1 only (the "Licence"), appearing in the file LICENCE.md       *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: Pedro Calado                                                     *
//***************************************************************************

#ifndef LUPISMODEL_HPP_
#define LUPISMODEL_HPP_

#include <labust/simulation/matrixfwd.hpp>
#include <ros/ros.h>

#include <cstring>
#include <cmath>

#include <Eigen/Dense>
//#include <DUNE/Config.hpp>
//#include <DUNE/Math.hpp>

// ISO C++ 98 headers.


// DUNE headers.
//#include <DUNE/Config.hpp>
//#include <DUNE/Math.hpp>

namespace labust
{
  namespace simulation
  {
    // Export DLL Symbol.
    //class DUNE_DLL_SYM AUVModel;


    struct ModelParameters
    {
      double mass;
      double density;
      double volume;
      double motor_friction;
      double max_thrust;
      double gravity;
      Eigen::VectorXd cog;
      Eigen::VectorXd addedmass;
      Eigen::VectorXd inertia;
      Eigen::VectorXd linear_drag;
      Eigen::VectorXd quadratic_drag;
      Eigen::VectorXd lift;
      Eigen::VectorXd fin_lift;

      ModelParameters(){
//          lauv_parameters.addedmass << -1, -16, -16, 0, 0, 0;
//  		lauv_parameters.cog << 0, 0, 0.01;
//  		lauv_parameters.density = 1020.0;
//  		lauv_parameters.fin_lift << 9.6, -9.6, 0.91, -3.84, -3.84;
//  		lauv_parameters.gravity = 9.81;
//  		lauv_parameters.inertia << 0.04, 2.1, 2.1;
//  		lauv_parameters.lift << -20.6, 3.84, -20.6, -3.84, -6, -1.53, 6, -1.53;
//  		lauv_parameters.linear_drag << -2.4, -23, -23, -0.3, -9.7, -9.7, 11.5, -11.5, 3.1, -3.1;
//  		lauv_parameters.mass = 18.054;
//  		lauv_parameters.max_thrust = 10.0;
//  		lauv_parameters.motor_friction = 0.006;
//  		lauv_parameters.quadratic_drag << -2.4, -80, -80, -0.0006, -9.1, -9.1, -0.3, 0.3, -1.5, 1.5;
//  		lauv_parameters.volume = 0.0177;
    	addedmass.resize(6);
        addedmass << -1, -16, -16, 0, 0, 0;
        cog.resize(3);
  		cog << 0, 0, 0.01;
  		density = 1020.0;
  		fin_lift.resize(5);
  		fin_lift << 9.6, -9.6, 0.91, -3.84, -3.84;
  		gravity = 9.81;
  		inertia.resize(3);
  		inertia << 0.04, 2.1, 2.1;
  		lift.resize(8);
  		lift << -20.6, 3.84, -20.6, -3.84, -6, -1.53, 6, -1.53;
  		linear_drag.resize(10);
  		linear_drag << -2.4, -23, -23, -0.3, -9.7, -9.7, 11.5, -11.5, 3.1, -3.1;
  		mass = 18.054;
  		max_thrust = 10.0;
  		motor_friction = 0.006;
  		quadratic_drag.resize(10);
  		quadratic_drag << -2.4, -80, -80, -0.0006, -9.1, -9.1, -0.3, 0.3, -1.5, 1.5;
  		volume = 0.0177;

      }
    };

    class AUVModel
    {
    public:
		typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix;
		typedef Eigen::Matrix<double, Eigen::Dynamic, 1> vector;

      //! Constructor
      AUVModel(ModelParameters& param);

      //! Destructor
      ~AUVModel();

      //! Routine to compute the next step
      matrix
      step(const matrix& nu_dot, const matrix& nu, const matrix& eta);

      //! Routine to compute the next step, yet compute the acceleration instead of forces
      matrix
      stepInv(const matrix& tau, const matrix& nu, const matrix& eta);

      //! Routine to compute the next step, yet compute the acceleration instead of forces
      //! this time using fin deflections and xyz forces
      matrix
      stepInv(const matrix& xyz, const matrix& deflections, const matrix& nu, const matrix& eta);

      //! Routine to compute the next step, yet compute the acceleration instead of forces
      //! this time using the thruster actuation and servo positions
      matrix
      stepInv(double thruster_act, const matrix& servo_pos, const matrix& nu, const matrix& eta);

      //! Get the roll fin lift coefficient
      inline double
      getRollFinLift(void)
      {
        return m_fin_lift(2);
      }

      //! Get the pitch fin lift coefficient
      inline double
      getPitchFinLift(void)
      {
        return m_fin_lift(3);
      }

      //! Get the yaw fin lift coefficient
      inline double
      getYawFinLift(void)
      {
        return m_fin_lift(4);
      }

    private:
      //! Computes added mass and inertia
      matrix
      computeM(void);

      //! Computes quadratic damping matrix
      matrix
      computeD(const matrix& nu);

      //! Computes lift matrix
      matrix
      computeL(const matrix& nu);

      //! Computes vector of restoring forces g
      matrix
      computeG(const matrix& eta);

      //! Computes rigid body coriolis and centripetal matrix
      matrix
      computeC(const matrix& nu);

      //! Compute the resulting tau using thruster actuation and servo positions
      matrix
      computeTau(double speed_u, double thruster_act, const matrix& servo_pos);

      /**
       * The function computes the skew-symmetric matrix from a given vector.
       * The function operates with vector size 3.
       *
       * \param vec The desired vector of size 3.
       * \tparam in The input matrix type.
       * \return The calculated 3x3 skew symmetric matrix.
       */

      matrix
          skew(double data[3])
          {
            matrix3 m;
            m(0, 1) = -data[2];
            m(0, 2) = data[1];
            m(1, 0) = data[2];
            m(1, 2) = -data[0];
            m(2, 0) = -data[1];
            m(2, 1) = data[0];

            return m;
          }

          matrix
          skew(const matrix& a)
          {
            //if (a.isEmpty())
            //  throw matrix::Error("Trying to access an empty matrix!");

            //if (!((a.m_nrows == 1 && a.m_ncols == 3) || (a.m_nrows == 3 && a.m_ncols == 1)))
              //throw Matrix::Error("Matrix must be 3x1 or 1x3 to create a skew symmetrical!");

            double data[3] = {a(0), a(1), a(2)};
            return skew(data);
          }



      //! Members
      //! Model's mass
      double m_mass;
      //! Model's density
      double m_density;
      //! Model's volume
      double m_volume;
      //! Model's motor friction coefficient
      double m_motor_friction;
      //! Model's max thrust force
      double m_max_thrust;
      //! gravity
      double m_gravity;
      //! Center of gravity's coordinates
      matrix m_cog;
      //! Added mass coeficients
      matrix m_addedmass;
      //! Intertia coeficients
      matrix m_inertia;
      //! Model's matrix of mass moments and added inertia
      matrix m_matrix_mass;
      //! Model's linear damping coefficients
      matrix m_ldrag;
      //! Model's quadratic drag
      matrix m_qdrag;
      //! Model's lift coefficients
      matrix m_lift;
      //! Model's fin lift coefficients
      matrix m_fin_lift;
    };



    //using std::sin;
    //using std::cos;

    //! Constructor.
    AUVModel::AUVModel(ModelParameters& param):
      m_mass(param.mass),
      m_density(param.density),
      m_volume(param.volume),
      m_motor_friction(param.motor_friction),
      m_max_thrust(param.max_thrust),
      m_cog(param.cog),
      m_addedmass(param.addedmass),
      m_inertia(param.inertia),
      m_ldrag(param.linear_drag),
      m_qdrag(param.quadratic_drag),
      m_lift(param.lift),
      m_fin_lift(param.fin_lift),
      m_gravity(param.gravity)

    {
      // compute matrix M which does not change with time
      m_matrix_mass = computeM();
    }

    //! Destructor.
    AUVModel::~AUVModel(void)
    { }

    AUVModel::matrix
    AUVModel::step(const matrix& nu_dot, const matrix& nu, const matrix& eta)
    {
      return m_matrix_mass * nu_dot + computeC(nu) * nu + computeD(nu) * nu + computeL(nu) * nu + computeG(eta);
    }

    AUVModel::matrix
    AUVModel::stepInv(const matrix& tau, const matrix& nu, const matrix& eta)
    {
      return m_matrix_mass.inverse() * (tau - computeC(nu) * nu - computeD(nu) * nu - computeL(nu) * nu - computeG(eta));
    }

    AUVModel::matrix
    AUVModel::stepInv(const matrix& xyz, const matrix& deflections, const matrix& nu, const matrix& eta)
    {
      Eigen::VectorXd tau(Eigen::VectorXd::Zero(6));

      tau(0) = xyz(0);
      tau(1) = xyz(1) + m_fin_lift(0) * nu(0) * nu(0) * deflections(2);
      tau(2) = xyz(2) + m_fin_lift(1) * nu(0) * nu(0) * deflections(1);

      tau(3) = m_fin_lift(2) * nu(0) * nu(0) * deflections(0);
      tau(4) = m_fin_lift(3) * nu(0) * nu(0) * deflections(1);
      tau(5) = m_fin_lift(4) * nu(0) * nu(0) * deflections(2);

      return stepInv(tau, nu, eta);
    }

    AUVModel::matrix
    AUVModel::stepInv(double thruster_act, const matrix& servo_pos, const matrix& nu, const matrix& eta)
    {
      matrix tau = computeTau(nu(0), thruster_act, servo_pos);
      return stepInv(tau, nu, eta);
    }

    //! Computes matrix of added mass and inertia
    AUVModel::matrix
    AUVModel::computeM(void)
    {
      // added mass matrix
      // matrix Ma = -matrix(&m_addedmass(0), 6);
      matrix Ma;
      Ma =-1*m_addedmass.asDiagonal();


      // mass and cog submatrix
      matrix cog = -1*skew(m_cog) * m_mass;

      // inertia submatrix
      //matrix I(&m_inertia(0), 3);
      matrix I;
      I = m_inertia.asDiagonal();

      //matrix Mrb = matrix(3) * m_mass;
      //Mrb.vertCat(-cog);
      //Mrb.horzCat(cog.vertCat(I));

      matrix Mrb;
      Mrb.resize(6,6);
      Mrb.setZero();

      Mrb.block<3,3>(0,0) = m_mass*Eigen::Matrix3d::Identity();
      Mrb.block<3,3>(3,0) = -1*cog;
      Mrb.block<3,3>(0,3) = cog;
      Mrb.block<3,3>(3,3) = I;

      return Mrb + Ma;
    }

    //! Computes coriolis and centripetal matrix in the skew symmetric form
    AUVModel::matrix
    AUVModel::computeC(const matrix& nu)
    {
      // CA
      //matrix a = (m_addedmass & nu).get(0, 2, 0, 0);
      //matrix b = (m_addedmass & nu).get(3, 5, 0, 0);
      matrix nu1 = nu.block<3,1>(0, 0);
      matrix nu2 = nu.block<3,1>(3, 0);



      	  matrix Ma;
      	  Ma =-1*m_addedmass.asDiagonal();


    	matrix3 M11=Ma.block<3,3>(0,0);
    	matrix3 M12=Ma.block<3,3>(0,3);
    	matrix3 M21=Ma.block<3,3>(3,0);
    	matrix3 M22=Ma.block<3,3>(3,3);

    	      matrix ca;
    	      ca.resize(6,6);
    	      ca.setZero();

    	ca.block<3,3>(0,3) = skew(M11*nu1 + M12*nu2);
    	ca.block<3,3>(3,0) = skew(M11*nu1 + M21*nu2);
    	ca.block<3,3>(3,3) = skew(M21*nu1 + M22*nu2);

      //matrix ca = matrix(3, 3, 0.0).horzCat(skew(a));
      //ca.vertCat(skew(a).horzCat(skew(b)));

//      matrix ca;
//      ca.resize(6,6);
//      ca.setZero();
//
//      //Mrb.block<3,3>(0,0) = m_mass*Eigen::Matrix3d::Identity();
//      ca.block<3,3>(0,3) = skew(a);
//      ca.block<3,3>(3,0) = skew(a);
//      ca.block<3,3>(3,3) = skew(b);


//      // CRB
//      matrix skew_cog = skew(m_cog);
//
//      //matrix skew_v1 = skew(nu.get(0, 2, 0, 0));
//      //matrix skew_v2 = skew(nu.get(3, 5, 0, 0));
//
//      matrix skew_v1 = skew(nu1);
//      matrix skew_v2 = skew(nu2);
//
//      matrix crb(3, 3, 0.0);
//      crb.vertCat(-m_mass * skew_v1 + m_mass * skew_cog * skew_v2);
//
//      matrix crb2 = -m_mass * skew_v1 - m_mass * skew_v2 * skew_cog;
//      crb2.vertCat(-skew(matrix(&m_inertia(0), 3) * nu.get(3, 5, 0, 0)));
//
//      crb.horzCat(crb2);

      matrix Io;
      Io = m_inertia.asDiagonal();


      matrix crb;
      crb.resize(6,6);
      crb.setZero();

		crb.block<3,3>(0,3) = -m_mass*skew(nu1) - m_mass*skew(nu2)*skew(m_cog);
		crb.block<3,3>(3,0) = -m_mass*skew(nu1) + m_mass*skew(m_cog)*skew(nu2);
		crb.block<3,3>(3,3) = -skew(Io*nu2);

      return ca + crb;
    }

    //! Routine to compute matrix D using state nu
    AUVModel::matrix
    AUVModel::computeD(const matrix& nu)
    {
      double u = nu(0);
      double v = nu(1);
      double w = nu(2);
      double p = nu(3);
      double q = nu(4);
      double r = nu(5);

      // initialize linear drag matrix with diagonal terms
      //matrix dl = matrix(&m_ldrag(0), 6);
      matrix dl = m_ldrag.block<6,1>(0,0).asDiagonal();
      // initialize quadratic drag matrix
      //matrix dq = matrix(6);
      matrix dq;
      dq.resize(6,6);
      dq.setZero();

      // linear drag coupling terms
      dl(1, 5) = m_ldrag(6);
      dl(2, 4) = m_ldrag(7);
      dl(4, 2) = m_ldrag(8);
      dl(5, 1) = m_ldrag(9);

      // quadratic diagonal terms
      dq(0, 0) = m_qdrag(0) * std::abs(u);
      dq(1, 1) = m_qdrag(1) * std::abs(v);
      dq(2, 2) = m_qdrag(2) * std::abs(w);
      dq(3, 3) = m_qdrag(3) * std::abs(p);
      dq(4, 4) = m_qdrag(4) * std::abs(q);
      dq(5, 5) = m_qdrag(5) * std::abs(r);
      // quadratic coupling terms
      dq(1, 5) = m_qdrag(6) * std::abs(v);
      dq(2, 4) = m_qdrag(7) * std::abs(w);
      dq(4, 2) = m_qdrag(8) * std::abs(q);
      dq(5, 1) = m_qdrag(9) * std::abs(r);

      return -(dl + dq);
    }

    //! Routine to compute lift forces matrix
    AUVModel::matrix
    AUVModel::computeL(const matrix& nu)
    {
      //matrix lift(6, 6, 0.0);
      matrix lift;
      lift.resize(6,6);
      lift.setZero();

      lift(1, 1) = m_lift(0);
      lift(2, 2) = m_lift(1);
      lift(4, 4) = m_lift(2);
      lift(5, 5) = m_lift(3);
      lift(1, 5) = m_lift(4);
      lift(2, 4) = m_lift(5);
      lift(4, 2) = m_lift(6);
      lift(5, 1) = m_lift(7);

      return -lift * nu(0);
    }

    //! Routine to compute vector of restoring forces g
    AUVModel::matrix
    AUVModel::computeG(const matrix& eta)
    {
      double phi = eta(3);
      double theta = eta(4);
      double W = m_mass * m_gravity;
      double B = m_density * m_gravity * m_volume;

      Eigen::VectorXd g;
      g.resize(6);
      g << (W - B) * sin(theta),
        -(W - B) * cos(theta) * sin(phi),
        -(W - B) * cos(theta) * cos(phi),
        -m_cog(1) * W * cos(theta) * cos(phi) + m_cog(2) * W * cos(theta) * sin(phi),
        m_cog(2) * W * sin(theta) + m_cog(0) * W * cos(theta) * cos(phi),
        -m_cog(0) * W * cos(theta) * sin(phi) - m_cog(1) * W * sin(theta);

      return g;
    }

    AUVModel::matrix
    AUVModel::computeTau(double speed_u, double thruster_act, const matrix& servo_pos)
    {
      //matrix tau(6, 1, 0.0);
      //matrix deflections(3, 1, 0.0);

        matrix tau;
        tau.resize(6,6);
        tau.setZero();

        matrix deflections;
        deflections.resize(6,6);
        deflections.setZero();



      deflections(0) = servo_pos(3) - servo_pos(0) + servo_pos(1) - servo_pos(2);
      deflections(1) = servo_pos(1) + servo_pos(2);
      deflections(2) = servo_pos(0) + servo_pos(3);

      tau(0) = thruster_act * m_max_thrust;
      tau(1) = m_fin_lift(0) * speed_u * speed_u * deflections(2);
      tau(2) = m_fin_lift(1) * speed_u * speed_u * deflections(1);
      tau(3) = m_fin_lift(2) * speed_u * speed_u * deflections(0) + m_motor_friction * tau(0);
      tau(4) = m_fin_lift(3) * speed_u * speed_u * deflections(1);
      tau(5) = m_fin_lift(4) * speed_u * speed_u * deflections(2);

      return tau;
    }
  }
}

/* LUPISMODEL_HPP_ */
#endif
