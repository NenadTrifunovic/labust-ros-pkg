/*********************************************************************
 * maneuverGenerator.hpp
 *
 *  Created on: Apr 3, 2014
 *      Author: Filip Mandic
 *
 ********************************************************************/

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014-2016, LABUST, UNIZG-FER
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

#ifndef MANEUVERGENERATOR_HPP_
#define MANEUVERGENERATOR_HPP_

#include <labust/mission/labustMission.hpp>
#include <labust_mission/xmlPrinter.hpp>

#include <boost/lexical_cast.hpp>
#include <Eigen/Dense>



using namespace labust::utils;

/*********************************************************************
 *** ManeuverGenerator class definition
 *********************************************************************/

namespace labust {
	namespace maneuver {

		class ManeuverGenerator {

		public:

			/*********************************************************
			 *** Class functions
			 ********************************************************/
			ManeuverGenerator(std::string xml_path);

			~ManeuverGenerator();

			void generateGo2Point_FA(
					double north,
					double east,
					double depth,
					double speed,
					double victory_radius);

			void generateStationKeeping(
					double north,
					double east,
					double depth,
					double heading);

			void generateRows(
					double north,
					double east,
					double depth,
					double speed,
					double victory_radius,
					double width,
					double length,
					double hstep,
					double alternationPercent,
					double curvOff,
					bool squareCurve,
					double bearing,
					double crossAngle,
					bool invertY);


			/*********************************************************
			 *** Full primitive generation prototypes
			 ********************************************************/

			void generateGo2Point(
					bool enable_fully_actuated,
					double north,
					double east,
					double depth,
					double heading,
					double speed,
					double victory_radius,
					bool north_enable,
					bool east_enable,
					bool depth_enable,
					bool heading_enable,
					bool altitude_enable,
					std::string heading_topic,
					std::string speed_topic);

			void generateDynamicPositioning(
					double north,
					double east,
					double depth,
					double heading,
					bool north_enable,
					bool east_enable,
					bool depth_enable,
					bool heading_enable,
					bool altitude_enable,
					bool target_topic_enable,
					bool track_heading_enable,
					bool point_to_target,
					std::string target_topic,
					std::string heading_topic
					);

			void generatePointer(
					double radius,
					double vertical_offset,
					double guidance_target_x,
					double guidance_target_y,
					double guidance_target_z,
					bool guidance_enable,
					bool wrapping_enable,
					bool streamline_orientation,
					std::string guidance_topic,
					std::string radius_topic);

			void generateDocking(
				  bool docking_action,
				  double docking_slot,
          double search_yaw_rate,
          double max_yaw_rate,
          double max_surge_speed,
          double surge_stdev					
								);



			/*********************************************************
			 *** Helper functions
			 ********************************************************/

		private:

			void writePrimitives(int primitiveID, std::vector<Eigen::Vector4d> points, double depth, double heading, double speed, double victoryRadius);

			std::vector<Eigen::Vector4d> calcRIPatternPoints(double width, double hstep,
								double alternationPercent, double curvOff, bool squareCurve, double bearingRad);

			std::vector<Eigen::Vector4d> calcCrossHatchPatternPoints(double width, double hstep,
								double curvOff, bool squareCurve, double bearingRad);

			std::vector<Eigen::Vector4d> calcRowsPoints(double width, double length, double hstep,
								double alternationPercent, double curvOff, bool squareCurve, double bearingRad,
								double crossAngleRadians);

			std::vector<Eigen::Vector4d> calcRowsPoints(double width, double length, double hstep,
								double alternationPercent, double curvOff, bool squareCurve, double bearingRad,
								double crossAngleRadians, bool invertY);

			std::pair<double,double> rotate(double angleRadians, double x, double y, bool clockwiseRotation);


			/*********************************************************
			 *** Class variables
			 ********************************************************/
		public:
			WriteXML writeXML;
		};


		ManeuverGenerator::ManeuverGenerator(std::string xml_path):writeXML(xml_path)
		{

		}

		ManeuverGenerator::~ManeuverGenerator(){}

		/*** North and east parameters must include offset in case of relative mission start. */
		void ManeuverGenerator::generateGo2Point_FA(double north, double east, double depth, double speed, double victory_radius)
		{
			generateGo2Point(true,north,east,depth,0,speed,victory_radius,true,true,true,false,false,"","");
		}

		void ManeuverGenerator::generateRows(double north, double east, double depth, double speed, double victory_radius, double width, double length, double hstep, double alternationPercent,
				double curvOff, bool squareCurve, double bearing, double crossAngle, bool invertY)
		{
			/* Generate maneuver points */
			std::vector<Eigen::Vector4d> tmpPoints;
			tmpPoints = calcRowsPoints(width, length, hstep,
						alternationPercent/100, curvOff, squareCurve, bearing*M_PI/180,
						crossAngle*M_PI/180, invertY);

			/* For each point subtract offset and add start point */
			for(std::vector<Eigen::Vector4d>::iterator it = tmpPoints.begin(); it != tmpPoints.end(); ++it)
			{
				Eigen::Vector4d vTmp = *it;
				vTmp[X] += north;
				vTmp[Y] += east;

				*it = vTmp;
			}

			writePrimitives(go2point, tmpPoints, depth, 0, speed, victory_radius); /* heading, speed, victoryRadius */
		}

		void ManeuverGenerator::generateStationKeeping(double north, double east, double depth, double heading)
		{
			generateDynamicPositioning(north,east,depth,heading,true,true,true,true,false,false,false,false,"","");
		}



		/*********************************************************
		 *** Full primitive generation prototypes
		 ********************************************************/

		/*** North and east parameters must include offset in case of relative mission start. */
		void ManeuverGenerator::generateGo2Point(
				bool enable_fully_actuated,
				double north,
				double east,
				double depth,
				double heading,
				double speed,
				double victory_radius,
				bool north_enable,
				bool east_enable,
				bool depth_enable,
				bool heading_enable,
				bool altitude_enable,
				std::string heading_topic,
				std::string speed_topic)
		{
			std::vector<std::string> data;
			data.push_back(boost::lexical_cast<std::string>(enable_fully_actuated));
			data.push_back(boost::lexical_cast<std::string>(north));
			data.push_back(boost::lexical_cast<std::string>(east));
			data.push_back(boost::lexical_cast<std::string>(depth));
			data.push_back(boost::lexical_cast<std::string>(heading));
			data.push_back(boost::lexical_cast<std::string>(speed));
			data.push_back(boost::lexical_cast<std::string>(victory_radius));
			data.push_back(boost::lexical_cast<std::string>(north_enable));
			data.push_back(boost::lexical_cast<std::string>(east_enable));
			data.push_back(boost::lexical_cast<std::string>(depth_enable));
			data.push_back(boost::lexical_cast<std::string>(heading_enable));
			data.push_back(boost::lexical_cast<std::string>(altitude_enable));
			data.push_back("#"+heading_topic);
			data.push_back("#"+speed_topic);

			writeXML.addPrimitive(go2point,data);

		}

		void ManeuverGenerator::generateDynamicPositioning(
				double north,
				double east,
				double depth,
				double heading,
				bool north_enable,
				bool east_enable,
				bool depth_enable,
				bool heading_enable,
				bool altitude_enable,
				bool target_topic_enable,
				bool track_heading_enable,
				bool point_to_target,
				std::string target_topic,
				std::string heading_topic)
			{
				std::vector<std::string> data;
				data.push_back(boost::lexical_cast<std::string>(north));
				data.push_back(boost::lexical_cast<std::string>(east));
				data.push_back(boost::lexical_cast<std::string>(depth));
				data.push_back(boost::lexical_cast<std::string>(heading));
				data.push_back(boost::lexical_cast<std::string>(north_enable));
				data.push_back(boost::lexical_cast<std::string>(east_enable));
				data.push_back(boost::lexical_cast<std::string>(depth_enable));
				data.push_back(boost::lexical_cast<std::string>(heading_enable));
				data.push_back(boost::lexical_cast<std::string>(altitude_enable));
				data.push_back(boost::lexical_cast<std::string>(target_topic_enable));
				data.push_back(boost::lexical_cast<std::string>(track_heading_enable));
				data.push_back(boost::lexical_cast<std::string>(point_to_target));
				data.push_back("#"+target_topic);
				data.push_back("#"+heading_topic);

				writeXML.addPrimitive(dynamic_positioning,data);

			}

		void ManeuverGenerator::generatePointer(
				double radius,
				double vertical_offset,
				double guidance_target_x,
				double guidance_target_y,
				double guidance_target_z,
				bool guidance_enable,
				bool wrapping_enable,
				bool streamline_orientation,
				std::string guidance_topic,
				std::string radius_topic)
		{
			std::vector<std::string> data;
			data.push_back(boost::lexical_cast<std::string>(radius));
			data.push_back(boost::lexical_cast<std::string>(vertical_offset));
			data.push_back(boost::lexical_cast<std::string>(guidance_target_x));
			data.push_back(boost::lexical_cast<std::string>(guidance_target_y));
			data.push_back(boost::lexical_cast<std::string>(guidance_target_z));
			data.push_back(boost::lexical_cast<std::string>(double(guidance_enable)));
			data.push_back(boost::lexical_cast<std::string>(double(wrapping_enable)));
			data.push_back(boost::lexical_cast<std::string>(double(streamline_orientation)));
			data.push_back("#"+guidance_topic);
			data.push_back("#"+radius_topic);
			writeXML.addPrimitive(pointer,data);
		}

		void ManeuverGenerator::generateDocking(
				  bool docking_action,
				  double docking_slot,
          double search_yaw_rate,
          double max_yaw_rate,
          double max_surge_speed,
          double surge_stdev				
		)
		{
			std::vector<std::string> data;
			data.push_back(boost::lexical_cast<std::string>(docking_action));
			data.push_back(boost::lexical_cast<std::string>(docking_slot));
			data.push_back(boost::lexical_cast<std::string>(search_yaw_rate));
			data.push_back(boost::lexical_cast<std::string>(max_yaw_rate));
			data.push_back(boost::lexical_cast<std::string>(max_surge_speed));
			data.push_back(boost::lexical_cast<std::string>(surge_stdev));
			writeXML.addPrimitive(docking,data);
		}





		/*************************************************************
		 *** Helper functions
		 ************************************************************/

		void ManeuverGenerator::writePrimitives(int primitiveID, std::vector<Eigen::Vector4d> points, double depth, double heading, double speed, double victoryRadius)
		{
			switch(primitiveID)
			{
				case go2point:

					for(std::vector<Eigen::Vector4d>::iterator it = points.begin() ; it != points.end(); ++it)
					{
							//Eigen::Vector4d vTmp = *it;
							generateGo2Point_FA(double((*it)[X]), double((*it)[Y]), 0, speed, victoryRadius);
					}

					//ROS_ERROR("T1 = %f,%f, T2 = %f,%f, Heading = %f, Speed = %f, Victory radius = %f", CM->Xpos, CM->Ypos, newXpos, newYpos, newHeading, newSpeed, newVictoryRadius);
					break;

				case dynamic_positioning:

					//ROS_ERROR("T2 = %f,%f, Heading = %f", newXpos, newYpos, newHeading);

					break;

				case course_keeping:
					//ROS_ERROR("Course = %f, Heading = %f, Speed = %f", newCourse, newHeading, newSpeed);

					break;

				case follow:

					//ROS_ERROR("Course = %f, Speed = %f", newCourse, newSpeed);

					break;

				case none:

					break;

			}
		}


		std::vector<Eigen::Vector4d> ManeuverGenerator::calcRIPatternPoints(double width, double hstep,
				double alternationPercent, double curvOff, bool squareCurve, double bearingRad) {

			std::vector<Eigen::Vector4d> newPoints;

			double length = width;
			Eigen::Vector4d pointBaseB;
			pointBaseB << -length/2.0, -width/2.0, 0, -1;

			std::pair<double, double> res = rotate(bearingRad, pointBaseB[X], pointBaseB[Y], false);
		   // double[] res = AngleCalc.rotate(bearingRad, pointBaseB[X], pointBaseB[Y], false);
			Eigen::Vector4d pointBase1;
			pointBase1  << res.first, res.second, 0, -1;

			res = rotate(bearingRad+(-60*M_PI/180), pointBaseB[X], pointBaseB[Y], false);

			Eigen::Vector4d pointBase2;
			pointBase2 << res.first, res.second, 0, -1;

			res = rotate(bearingRad+(-120*M_PI/180), pointBaseB[X], pointBaseB[Y], false);

			Eigen::Vector4d pointBase3;
			pointBase3 << res.first, res.second, 0, -1;

			std::vector<Eigen::Vector4d> points1 = calcRowsPoints(width, width, hstep, 2-alternationPercent, curvOff,
					squareCurve, bearingRad, 0);

			for(std::vector<Eigen::Vector4d>::iterator it = points1.begin() ; it != points1.end(); ++it){

				Eigen::Vector4d vTmp = *it;
				vTmp[X] += pointBase1[X];
				vTmp[Y] += pointBase1[Y];

				*it = vTmp;
			}

			std::vector<Eigen::Vector4d> points2 = calcRowsPoints(width, width, hstep, 2-alternationPercent, curvOff,
							squareCurve, bearingRad + (-60*M_PI/180), 0);

			for(std::vector<Eigen::Vector4d>::iterator it = points2.begin() ; it != points2.end(); ++it){

				Eigen::Vector4d vTmp = *it;
				vTmp[X] += pointBase2[X];
				vTmp[Y] += pointBase2[Y];

				*it = vTmp;
			}

			std::vector<Eigen::Vector4d> points3 = calcRowsPoints(width, width, hstep, 2-alternationPercent, curvOff,
							squareCurve, bearingRad + (-120*M_PI/180), 0);

			for(std::vector<Eigen::Vector4d>::iterator it = points3.begin() ; it != points3.end(); ++it){

				Eigen::Vector4d vTmp = *it;
				vTmp[X] += pointBase3[X];
				vTmp[Y] += pointBase3[Y];

				*it = vTmp;
			}

			// Provjeri ovaj dio moguci problemi s pokazivacem !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			std::vector<Eigen::Vector4d>::iterator it;

			it = newPoints.end();
			newPoints.insert(it+1, points1.begin(), points1.end());
			it = newPoints.end();
			newPoints.insert(it+1, points2.begin(), points2.end());
			it = newPoints.end();
			newPoints.insert(it+1, points3.begin(), points3.end());

			return newPoints;
		}


		std::vector<Eigen::Vector4d> ManeuverGenerator::calcCrossHatchPatternPoints(double width, double hstep,
				double curvOff, bool squareCurve, double bearingRad) {

			std::vector<Eigen::Vector4d> newPoints;

			double length = width;

			Eigen::Vector4d pointBase1;
			pointBase1 << -length/2., -width/2., 0, -1;

			Eigen::Vector4d pointBase2;
			pointBase2 << -length/2., width/2., 0, -1;

			std::pair<double, double> res = rotate(bearingRad, pointBase1[X], pointBase1[Y], false);

			pointBase1 << res.first, res.second, 0, -1;

			res = rotate(bearingRad, pointBase2[X], pointBase2[Y], false);

			pointBase2 << res.first, res.second, 0, -1;

			std::vector<Eigen::Vector4d> points1 = calcRowsPoints(width, width, hstep, 1, curvOff,
					squareCurve, bearingRad, 0);

			for(std::vector<Eigen::Vector4d>::iterator it = points1.begin() ; it != points1.end(); ++it){

				Eigen::Vector4d vTmp = *it;
				vTmp[X] += pointBase1[X];
				vTmp[Y] += pointBase1[Y];

				*it = vTmp;
			}

			std::vector<Eigen::Vector4d> points2 = calcRowsPoints(width, width, hstep, 1, curvOff,
							squareCurve, bearingRad + (-90*M_PI/180), 0);

			for(std::vector<Eigen::Vector4d>::iterator it = points2.begin() ; it != points2.end(); ++it){

				Eigen::Vector4d vTmp = *it;
				vTmp[X] += pointBase2[X];
				vTmp[Y] += pointBase2[Y];

				*it = vTmp;
			}

			// Provjeri ovaj dio moguci problemi s pokazivacem !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			std::vector<Eigen::Vector4d>::iterator it;

			it = newPoints.end();
			newPoints.insert(it+1, points1.begin(), points1.end());
			it = newPoints.end();
			newPoints.insert(it+1, points2.begin(), points2.end());

			return newPoints;
		}


		 std::vector<Eigen::Vector4d> ManeuverGenerator::calcRowsPoints(double width, double length, double hstep,
				double alternationPercent, double curvOff, bool squareCurve, double bearingRad,
				double crossAngleRadians) {
			return calcRowsPoints(width, length, hstep, alternationPercent, curvOff, squareCurve,
					bearingRad, crossAngleRadians, false);
		}


		 std::vector<Eigen::Vector4d> ManeuverGenerator::calcRowsPoints(double width, double length, double hstep,
				double alternationPercent, double curvOff, bool squareCurve, double bearingRad,
				double crossAngleRadians, bool invertY) {

			width = fabs(width);
			length = fabs(length);
			hstep = fabs(hstep);

			bool direction = true;

			std::vector<Eigen::Vector4d> newPoints;

			Eigen::Vector4d point;
			point<<-curvOff,0,0,-1;

			newPoints.push_back(point);

	// double x1;
			double x2;
			for (double y = 0; y <= width; y += hstep) {

				if (direction) {
	// x1 = -curvOff;
					x2 = length + curvOff;
				}
				else {
	// x1 = length + curvOff;
					x2 = -curvOff;
				}
				direction = !direction;

				double hstepDelta = 0;
				if (direction)
					hstepDelta = hstep * (1 - alternationPercent);
				//Eigen::Vector4d point;
				point << x2, y - hstepDelta, 0, -1;

				newPoints.push_back(point);

				if (y + hstep <= width) {
					double hstepAlt = hstep;
					if (!direction)
						hstepAlt = hstep * alternationPercent;

					point << x2 + (squareCurve ? 0 : 1) * (direction ? curvOff : -curvOff), y + hstepAlt, 0, -1;
					newPoints.push_back(point);
				}
			}

			for (std::vector<Eigen::Vector4d>::iterator it = newPoints.begin() ; it != newPoints.end(); ++it){


				//std::cout << ' ' << *it;

				Eigen::Vector4d vTmp = *it;
			   // double yTmp = vTmp[0];

				std::pair<double,double> res = rotate(-crossAngleRadians, vTmp[0], 0, false);
				vTmp[X] = res.first;
				vTmp[Y] = vTmp[Y] + res.second;
				if (invertY)
					vTmp[Y] = -vTmp[Y];
				res = rotate(bearingRad + (!invertY ? -1 : 1) * -crossAngleRadians, vTmp[X], vTmp[Y], false);
				vTmp[X] = res.first;
				vTmp[Y] = res.second;

				*it = vTmp;

			}

			return newPoints;
		}

		 /**
			 * XY Coordinate conversion considering a rotation angle.
			 *
			 * @param angleRadians angle
			 * @param x original x value on entry, rotated x value on exit.
			 * @param y original y value on entry, rotated y value on exit.
			 * @param clockwiseRotation clockwiseRotation rotation or not
			 */


		 std::pair<double,double> ManeuverGenerator::rotate(double angleRadians, double x, double y, bool clockwiseRotation) {

			double sina = sin(angleRadians);
			double cosa = cos(angleRadians);

			std::pair<double,double> xy;

			if (clockwiseRotation) {
				xy.first = x * cosa + y * sina;
				xy.second = -x * sina + y * cosa;
			}
			else {
				xy.first = x * cosa - y * sina;
				xy.second = x * sina + y * cosa;
			}
			return xy;
		}
	}
}

#endif /* MANEUVERGENERATOR_HPP_ */
