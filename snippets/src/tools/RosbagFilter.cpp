/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, LABUST, UNIZG-FER
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
 *  Author : Ivor Rendulic
 *  Created: 19.10.2015.
 *********************************************************************/
#include <ros/ros.h>
#include <labust/tools/RosbagFilter.hpp>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
using namespace labust::tools;


RosbagFilter::RosbagFilter() {}

void RosbagFilter::setOutputBag(const std::string& out_filename) {
  out_bag_name_ = out_filename;
}

void RosbagFilter::setInputBags(const std::vector<std::string>& in_bags) {
  in_bags_ = in_bags;
}

void RosbagFilter::setTopics(const std::vector<std::string>& topics) {
  topics_ = topics;
}

void RosbagFilter::setTime(const ros::Time& start_t, const ros::Time& end_t) {
  time_start = start_t;
  time_end = end_t;
}

void RosbagFilter::start() {
  bag_writer.setBag(out_bag_name_);
  bag_writer.open();
  for (int i=0; i<in_bags_.size(); ++i) {
    ROS_INFO("Opening bag %s.", in_bags_[i].c_str());
    RosbagReader bag_reader(in_bags_[i]);
    bag_reader.addTopics(topics_);
    bag_reader.setTime(time_start, time_end);
    bag_reader.open();
    while (!bag_reader.done()) {
      rosbag::MessageInstance m = bag_reader.nextMessageInstance();
      bag_writer.addMessage(m.getTopic(), m.getTime(), m);
    }
    ROS_INFO("Done with bag, closing.");
    bag_reader.close();
  }
  bag_writer.close();
}

int main(int argc, char **argv) { 
  std::string curr_flag = "";
  std::string out_bag_filename;
  std::vector<std::string> in_bags;
  std::vector<std::string> topics;
  ros::Time time_start = ros::TIME_MIN;
  ros::Time time_end = ros::TIME_MAX;
  for (int i=1; i<argc; ++i) {
    std::string curr_argv(argv[i]);
    if (curr_argv[0] == '-') {
      curr_flag = curr_argv;
      continue;
    }
    if (curr_flag == "-b") {
      in_bags.push_back(curr_argv);
    } else if (curr_flag == "-t") {
      topics.push_back(curr_argv);
    } else if (curr_flag == "-o") {
      out_bag_filename = curr_argv; 
    } else if (curr_flag == "-start_time") {
      time_start = ros::Time(strtod(argv[i], NULL));
    } else if (curr_flag == "-end_time") {
      time_end = ros::Time(strtod(argv[i], NULL));
    }
  }
  
  RosbagFilter filter;
  filter.setOutputBag(out_bag_filename);
  filter.setInputBags(in_bags);
  filter.setTopics(topics);
  filter.setTime(time_start, time_end);
  filter.start();
  ROS_INFO("Done.");
  return 0;
}
