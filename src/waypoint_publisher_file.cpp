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
#include <thread>
#include <chrono>

#include <Eigen/Geometry>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

bool sim_running = false;

static const int64_t kNanoSecondsInSecond = 1000000000;

void callback(const sensor_msgs::ImuPtr& /*msg*/) {
  sim_running = true;
}

class WaypointWithTime {
 public:
  WaypointWithTime()
      : waiting_time(0),
        position(0, 0, 0),
        yaw(0) {
  }
  // WaypointWithTime()
  //     : waiting_time(0) {
  // }

  WaypointWithTime(double t, float x, float y, float z, float _yaw)
      : position(x, y, z), yaw(_yaw), waiting_time(t) {
  }

  Eigen::Vector3d position;
  double yaw;
  double waiting_time;
};

int main(int argc, char** argv) {

  ros::init(argc, argv, "waypoint_publisher");
  ros::NodeHandle nh;

  ROS_INFO("Started waypoint_publisher.");

  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);

  if (args.size() != 2 && args.size() != 3) {
    ROS_ERROR("Usage: waypoint_publisher <waypoint_file>"
        "\nThe waypoint file should be structured as: space separated: wait_time [s] x[m] y[m] z[m] yaw[deg])");
    return -1;
  }

  std::vector<WaypointWithTime> waypoints;
  const float DEG_2_RAD = M_PI / 180.0;

  std::ifstream wp_file(args.at(1).c_str());

  if (wp_file.is_open()) {
    double t, x, y, z, yaw;
    // Only read complete waypoints.
    while (wp_file >> t >> x >> y >> z >> yaw) {
      waypoints.push_back(WaypointWithTime(t, x, y, z, yaw * DEG_2_RAD));
    }
    wp_file.close();
    ROS_INFO("Read %d waypoints.", (int )waypoints.size());
  }

  else {
    ROS_ERROR_STREAM("Unable to open poses file: " << args.at(1));
    return -1;
  }

  // The IMU is used, to determine if the simulator is running or not.
  ros::Subscriber sub = nh.subscribe("imu", 10, &callback);

  ros::Publisher wp_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;

  // Trying to unpause Gazebo for 10 seconds.
  // while (i <= 10 && !unpaused) {
  //   ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
  //   std::this_thread::sleep_for(std::chrono::seconds(1));
  //   unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  //   ++i;
  // }

  // if (!unpaused) {
  //   ROS_FATAL("Could not wake up Gazebo.");
  //   return -1;
  // }
  // else {
  //   ROS_INFO("Unpaused the Gazebo simulation.");
  // }


  ROS_INFO("Wait for simulation to become ready...");

  // while (!sim_running && ros::ok()) {
  //   ros::spinOnce();
  //   // ros::Duration(0.1).sleep();
  // }

  ROS_INFO("...ok");

  // Wait for 10s such that everything can settle and the mav flies to the initial position.
  ros::Duration(10).sleep();

  ROS_INFO("Start publishing waypoints.");

  trajectory_msgs::MultiDOFJointTrajectoryPtr msg(new trajectory_msgs::MultiDOFJointTrajectory);
  msg->header.stamp = ros::Time::now();
  msg->points.resize(waypoints.size());
  msg->joint_names.push_back("base_link");
  int64_t time_from_start_ns = 0;
  // for (size_t i = 0; i < waypoints.size(); ++i) {
  //   WaypointWithTime& wp = waypoints[i];

  //   mav_msgs::EigenTrajectoryPoint trajectory_point;
  //   trajectory_point.position_W = wp.position;
  //   trajectory_point.setFromYaw(wp.yaw);
  //   trajectory_point.time_from_start_ns = time_from_start_ns;

  //   time_from_start_ns += static_cast<int64_t>(wp.waiting_time * kNanoSecondsInSecond);

  //   mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &msg->points[i]);
  // }
  // wp_pub.publish(msg);

  // return 0;
  for (size_t i = 0; i < waypoints.size(); ++i) {

        trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
        trajectory_msg.header.stamp = ros::Time::now();
        Eigen::Vector3d desired_position(waypoints[i].position.x(), waypoints[i].position.y(), waypoints[i].position.z());
        double desired_yaw = waypoints[i].yaw;
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);

        ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",       
                 nh.getNamespace().c_str(),
                 desired_position.x(),
                 desired_position.y(),
                 desired_position.z());
        // wp_pub.publish(trajectory_msg);
        // ROS_INFO_STREAM(trajectory_msg);
         // Wait for t seconds to let the Gazebo GUI show up.
        double t = waypoints[i].waiting_time;
        // time_from_start_ns += static_cast<int64_t>(t * kNanoSecondsInSecond);
        
        ROS_INFO("ROS is sleeping");
        wp_pub.publish(trajectory_msg);
        ros::Duration(t).sleep();
        // ROS_INFO("THIS IS WAYPOINT");
        // ROS_INFO_STREAM( i);
  //       ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",       
  //          nh.getNamespace().c_str(),
  //          desired_position.x(),
  //          desired_position.y(),
  //          desired_position.z());


  }
  ros::spin();
}
