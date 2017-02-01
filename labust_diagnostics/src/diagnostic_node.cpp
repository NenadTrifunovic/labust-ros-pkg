/*
 * diagnostic_node.cpp
 *
 *  Created on: Jul 19, 2016
 *      Author: filip
 */

#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <ros/ros.h>

#include <labust/diagnostics/DiagnosticHandler.h>





/*********************************************************************
 ***  Main function
 ********************************************************************/

  int main(int argc, char** argv)
  {
  	ros::init(argc, argv, "diagnostic_node");

  	ros::NodeHandle nh,ph("~");

  	labust::diagnostic::DiagnosticHandler DH;

//    int i;
//    printf ("Checking if processor is available...");
//    if (system(NULL)) puts ("Ok");
//      else exit (EXIT_FAILURE);
//    printf ("Executing command DIR...\n");
//    i=system ("rostopic list");
//    printf ("The value returned was: %d.\n",i);

  	//std::cout << "pocetak " << exec("rostopic list") << " kraj" << std::endl;

  	// Wait for  system intitalization.
  	ros::Duration(10.0).sleep();


  	DH.subscribeTopics();

  	double Td = 5.0;
  	ros::Rate rate(1/Td);

  	while(ros::ok())
  	{
  		DH.checkStatus();

  		ros::spin();
  		rate.sleep();
  	}



  	ros::spin();
  	return 0;
  }
