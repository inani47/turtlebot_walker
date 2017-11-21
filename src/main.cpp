/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2017, Pranav Inani
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *  list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 *  @file main.cpp
 *
 *  @brief Implementation of walker node and ROS functionalities
 *
 *  @author Pranav Inani
 *  @copyright 2017
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "walker/walker.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "walker");
  ros::NodeHandle n;
  // sleep for 5 seconds while gazebo starts
  ros::Duration(5).sleep();
  Walker walk;
  ros::Subscriber subLaserScan;
  ros::Publisher pub;
  geometry_msgs::Twist msg;
  subLaserScan = n.subscribe("/scan", 1000, &Walker::callback, &walk);
  pub = n.advertise < geometry_msgs::Twist
      > ("/mobile_base/commands/velocity", 100);
  ros::Rate loop_rate(2);
  while (n.ok()) {
    // Initialize the twist messsage
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;
    if (walk.getObstDist() > .75) {
      //  Linear motion in forward direction
      msg.linear.x = 0.1;
      ROS_INFO_STREAM("Moving Forward");
    } else {
      //  2D Rotation of about 90 degress
      msg.angular.z = -0.8;
      ROS_WARN_STREAM("OBSTACLE DETECTED! Turning");
    }
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}

