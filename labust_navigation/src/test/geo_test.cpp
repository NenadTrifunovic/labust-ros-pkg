/*
 * eigen_test.cpp
 *
 *  Created on: Oct 16, 2013
 *      Author: dnad
 */
#include <labust/tools/GeoUtilities.hpp>
#include <iostream>

int main(int argc, char* argv[])
{
  Eigen::Vector3d geoInit(16,42,1000);

  Eigen::Vector3d xyz = labust::tools::geodetic2ecef(geoInit);
  std::cout<<"Origin, GEO->ECEF:"<<xyz.transpose()<<std::endl;
  Eigen::Vector3d ned = labust::tools::ecef2ned(xyz, geoInit);
  std::cout<<"Origin, ECEF->NED:"<<ned.transpose()<<std::endl;
  Eigen::Vector3d xyz2 = labust::tools::ned2ecef(ned, geoInit);
  std::cout<<"Origin, NED->ECEF:"<<xyz2.transpose()<<std::endl;
  Eigen::Vector3d geoInit2 = labust::tools::ecef2geodetic(xyz2);
  std::cout<<"Origin, ECEF->GEO:"<<geoInit2.transpose()<<std::endl;

  std::cout<<std::endl;
  Eigen::Vector3d nedR(1000, 5000, 1000);
  std::cout<<"Random NED point: n="<<nedR(0)<<", e="<<nedR(1)<<", d="<<nedR(2)<<std::endl;

  Eigen::Vector3d xyz3 = labust::tools::ned2ecef(nedR, geoInit);
  std::cout<<"Random, NED->ECEF:"<<xyz3.transpose()<<std::endl;
  Eigen::Vector3d geo3 = labust::tools::ecef2geodetic(xyz3);
  std::cout<<"Random, ECEF->GEO:"<<geo3.transpose()<<std::endl;
  Eigen::Vector3d xyz4 = labust::tools::geodetic2ecef(geo3);
  std::cout<<"Random, GEO->ECEF:"<<xyz4.transpose()<<std::endl;
  Eigen::Vector3d ned4 = labust::tools::ecef2ned(xyz4, geoInit);
  std::cout<<"Random, ECEF->NED:"<<ned4.transpose()<<std::endl;

	return 0;
}





