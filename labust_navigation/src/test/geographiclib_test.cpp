/*
 * eigen_test.cpp
 *
 *  Created on: Oct 16, 2013
 *      Author: dnad
 */
#include <labust/tools/GeoUtilities.hpp>
#include <labust/tools/conversions.hpp>
#include <labust/math/NumberManipulation.hpp>
#include <iostream>
#include <exception>
#include <cmath>
#include <ctime>
#include <boost/date_time.hpp>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/MagneticModel.hpp>

void compare_methods(double olat, double olon, double oh, double lat, double lon, double h)
{
	//Geo -> NED
	//Old conversion GEO->NED approx.
	std::pair<double, double >posxy =	labust::tools::deg2meter(lat - olat, lon - olon, olat);

	//Intermidiate converison GEO->ECEF->NED
	Eigen::Vector3d geoInit(olon, olat, oh), vehPos(lon,lat,h);
	Eigen::Vector3d xyz = labust::tools::geodetic2ecef(geoInit);
	Eigen::Vector3d vhc = labust::tools::geodetic2ecef(vehPos);
	Eigen::Vector3d ned = labust::tools::ecef2ned(xyz, geoInit);
	Eigen::Vector3d vned = labust::tools::ecef2ned(vhc, geoInit);

	//GeographicLib conversion
	double oX,oY,oZ,X,Y,Z,on,oe,od,n,e,d;
	try
	{
		using namespace GeographicLib;
		const Geocentric& earth = Geocentric::WGS84;
		LocalCartesian proj(olat, olon, oh, earth);
		proj.Forward(olat, olon, oh, on, oe, od);
		proj.Forward(lat, lon, h, n, e, d);
		earth.Forward(olat, olon, oh, oX, oY, oZ);
		earth.Forward(lat, lon, h, X, Y, Z);
		earth.Reverse(X, Y, Z, lat, lon, h);
	}
	catch (const std::exception& e)
	{
		std::cerr << "Caught exception: " << e.what() << "\n";

	}

	std::cout.precision(16);
	std::cout<<"Method 2 ECEF origin: x="<<xyz(0)<<", y="<<xyz(1)<<", z= "<<xyz(2)<<std::endl;
	std::cout<<"Method 3 ECEF origin: x="<<oX<<", y="<<oY<<"z = "<<oZ<<std::endl;
	std::cout<<"Method 2 ECEF vehicle: x="<<vhc(0)<<", y="<<vhc(1)<<", z= "<<vhc(2)<<std::endl;
	std::cout<<"Method 3 ECEF vehicle: x="<<X<<", y="<<Y<<", z= "<<Z<<std::endl;

	std::cout<<"Method 1 NED origin: x="<<0<<", y="<<0<<", z= "<<0<<std::endl;
	std::cout<<"Method 2 NED origin: x="<<ned(0)<<", y="<<ned(1)<<", z= "<<ned(2)<<std::endl;
	std::cout<<"Method 3 NED origin: x="<<on<<", y="<<oe<<", z= "<<od<<std::endl;
	std::cout<<"Method 1 NED vehicle: x="<<posxy.first<<", y="<<posxy.second<<", z= "<<-(h-oh)<<std::endl;
	std::cout<<"Method 2 NED vehicle: x="<<vned(0)<<", y="<<vned(1)<<", z= "<<vned(2)<<std::endl;
	std::cout<<"Method 3 ENU vehicle: x="<<n<<", y="<<e<<", z= "<<d<<std::endl;
	Eigen::Vector3d venu(n,e,d);
	Eigen::Quaternion<double> q;
	labust::tools::quaternionFromEulerZYX(M_PI,0,M_PI/2,q);
	vned = q.toRotationMatrix()*venu;
	std::cout<<"Method 3 NED vehicle: x="<<vned(0)<<", y="<<vned(1)<<", z= "<<vned(2)<<std::endl;
	std::cout<<"ENU->NED quaterion:"<<q.x()<<", "<<q.y()<<", "<<q.z()<<", "<<q.w()<<std::endl;
	q = q.inverse();
	std::cout<<"ENU->NED quaterion:"<<q.x()<<", "<<q.y()<<", "<<q.z()<<", "<<q.w()<<std::endl;
	std::cout<<"Method 3 reverse: lat="<<lat<<", lon="<<lon<<", h= "<<h<<std::endl;
}

void magnetic_declination(double lat, double lon, double h)
{
	using namespace GeographicLib;
  MagneticModel mag("wmm2015","/usr/share/geographiclib/magnetic");
  double Bx, By, Bz;
  using namespace boost::posix_time;
  double days = second_clock::local_time().date().day_of_year();
  double year = second_clock::local_time().date().year() + days/356.25;
  std::cout << "Current year:" << year << std::endl;
  mag(year, lat,lon, h, Bx, By, Bz);
  double H, F, D, I;
  MagneticModel::FieldComponents(Bx, By, Bz, H, F, D, I);
  std::cout<<"Current magnetic declination: "<< D <<std::endl;
}

int main(int argc, char* argv[])
{
	//Define the random geodetic point as LTP origin
	double olat(41.93),olon(15.99),oh(1000);
	//Define the random geodetic point of object
	double lat(42), lon(16), h(500);

	compare_methods(olat,olon,oh,lat,lon,h);
	magnetic_declination(lat,lon,h);

	std::cout<<"Wrapped:"<<labust::math::wrapRad(-7)<<std::endl;
	std::cout<<"Fmod:"<<fmod(-7, 2*M_PI)<<std::endl;

	return 0;
}





