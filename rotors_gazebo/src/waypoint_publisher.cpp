/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <fstream>
#include <iostream>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <string>
#include "std_msgs/Float32MultiArray.h"
#include "rotors_gazebo/Num.h"

double global_array_control[12]; //ugly change later
double global_time=20;

void algorithmCallback(const nav_msgs::OdometryConstPtr& odometry_msg_k) {

  double x, y, z; // (x,y,z) triplet holds a 3D position information
  x = odometry_msg_k->pose.pose.position.x;
  y = odometry_msg_k->pose.pose.position.y;
  z = odometry_msg_k->pose.pose.position.z;

  // IMPLEMENT HERE
}

void simpleLPCallback(const rotors_gazebo::Num& data) {
  for(int i = 0; i<12; i++) {
    global_array_control[i] = data.data[i]/global_time;
  }
}

/*void storeRefineCallback(const std_msgs::Float32MultiArray& unrefined_control) {
  double constrain_factor=1.5;
  double alt_time_factor = 0.2;

  if(ros::this_node::getName() == "/firefly1/waypoint_publisher") {
    for(int i = 0; i<3; i++) {
      global_array_control[i] = (unrefined_control.data[i] + constrain_factor)*alt_time_factor;
    }
  }
  else if (ros::this_node::getName() == "/firefly2/waypoint_publisher") {
    for(int i = 3; i<6; i++) {
      global_array_control[i] = (unrefined_control.data[i] + constrain_factor)*alt_time_factor;
    }
  }
  else if (ros::this_node::getName() == "/firefly3/waypoint_publisher") {
    for(int i = 6; i<9; i++) {
      global_array_control[i] = (unrefined_control.data[i] + constrain_factor)*alt_time_factor;
    }
  }
  else if (ros::this_node::getName() == "/firefly4/waypoint_publisher") {
    for(int i = 9; i<12; i++) {
      global_array_control[i] = ((unrefined_control.data[i]) + constrain_factor)*alt_time_factor;
    }
  }
}*/

int main(int argc, char** argv) {
  ros::init(argc, argv, "waypoint_publisher");
  ros::NodeHandle nh;
  ros::Publisher trajectory_pub =
  nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
    mav_msgs::default_topics::COMMAND_TRAJECTORY, 1);

  ros::Subscriber odometry_subscriber_k = nh.subscribe("odometry_sensor1/odometry", 1,
    algorithmCallback);
  //ros::Subscriber control_unrefined_k = nh.subscribe("/array_control", 1, storeRefineCallback);

  ros::Subscriber control_array = nh.subscribe("/array",1, simpleLPCallback);

  ROS_INFO("Started waypoint_publisher.");


  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);

  double delay;

  if (args.size() == 5) {
    delay = 1.0;
  } else if (args.size() == 6) {
    delay = std::stof(args.at(5));
  } else {
    ROS_ERROR("Usage: waypoint_publisher <x> <y> <z> <yaw_deg> [<delay>]\n");
    return -1;
  }

  const float DEG_2_RAD = M_PI / 180.0;

  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();

  Eigen::Vector3d desired_position(std::stof(args.at(1)), std::stof(args.at(2)),
   std::stof(args.at(3)));

  double desired_yaw = std::stof(args.at(4)) * DEG_2_RAD;

  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
    desired_yaw, &trajectory_msg);

  // Wait for some time to create the ros publisher.
  ros::Duration(delay).sleep();

  while (trajectory_pub.getNumSubscribers() == 0 && ros::ok()) {
    ROS_INFO("There is no subscriber available, trying again in 1 second.");
    ros::Duration(1.0).sleep();
  }

  trajectory_pub.publish(trajectory_msg);


  ros::Rate loop_rate(global_time);
  while(ros::ok()){
    trajectory_msg.header.stamp = ros::Time::now();
    Eigen::Vector3d desired_point_movement;
    //set a desired position movement
    if(ros::this_node::getName() == "/firefly1/waypoint_publisher") {
      desired_point_movement = Eigen::Vector3d(global_array_control[0],global_array_control[1],global_array_control[2]);
    }
    else if(ros::this_node::getName() == "/firefly2/waypoint_publisher") {
      desired_point_movement = Eigen::Vector3d(global_array_control[3],global_array_control[4],global_array_control[5]);
    }
    else if(ros::this_node::getName() == "/firefly3/waypoint_publisher") {
      //desired_point_movement = Eigen::Vector3d(global_array_control[6],global_array_control[7],global_array_control[8]);
      desired_point_movement = Eigen::Vector3d(0,0,0);
    }
    else {
      desired_point_movement = Eigen::Vector3d(global_array_control[9],global_array_control[10],global_array_control[11]);
      //desired_point_movement = Eigen::Vector3d(0.3,0.3,0.3);
    }

    desired_position = desired_position + desired_point_movement;
    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
      desired_yaw, &trajectory_msg);
    trajectory_pub.publish(trajectory_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
