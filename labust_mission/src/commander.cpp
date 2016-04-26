/*
 * commander.cpp
 *
 *  Created on: Apr 21, 2016
 *      Author: filip
 */

#include <labust/mission/commander.h>


/*********************************************************************
 ***  Main function
 ********************************************************************/

int main(int argc, char** argv)
{
	ros::init(argc, argv, "commander");
	labust::mission::Commander CMD;
	ros::spin();
	return 0;
}



