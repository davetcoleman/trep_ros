/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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

/* Author: Dave Coleman
   Desc:
*/

// ROS
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <geometry_msgs/Vector3.h>
#include "trep_ros/trep_ros.h"

int main(int argc, char** argv)
{
  ROS_INFO_STREAM_NAMED("trep_ros","Starting");

  ros::init(argc, argv, "trep_ros");

  // Load URDF to file
  std::ifstream in_urdf("baxter.urdf");
  std::string urdf_string((std::istreambuf_iterator<char>(in_urdf)), std::istreambuf_iterator<char>());

  std::ifstream in_srdf("baxter.srdf");
  std::string srdf_string((std::istreambuf_iterator<char>(in_srdf)), std::istreambuf_iterator<char>());

  // Error check
  if (srdf_string.empty())
  {
    ROS_ERROR_STREAM_NAMED("main","No SRDF string found");
    exit(1);
  }
  if (urdf_string.empty())
  {
    ROS_ERROR_STREAM_NAMED("main","No URDF string found");
    exit(1);
  }

  // Load the robot model
  robot_model_loader::RobotModelLoader::Options model_opts(urdf_string, srdf_string);
  model_opts.load_kinematics_solvers_ = false;
  robot_model_loader::RobotModelLoader rloader(model_opts);

  // Get the robot model from the loader
  const robot_model::RobotModelPtr robot_model = rloader.getModel();

  const std::string group_name = "right_arm";

  // Gravity
  geometry_msgs::Vector3 gravity_vector;
  gravity_vector.x = 0;
  gravity_vector.y = 0;
  gravity_vector.z = -9.81;

  // Initiate
  trep_ros::TrepROS tester(robot_model, group_name, gravity_vector);

  // Do Calc ----------------------------------------------------------------------
  tester.runTest();

  std::vector<double> joint_angles;
  for (std::size_t i = 0; i < 7; ++i)
  {
    joint_angles.push_back(0.0);
  }
  double payload;

  // get max payload
  /*
  unsigned int joint_saturated;
  if( !tester.getMaxPayload(joint_angles, payload, joint_saturated) )
    ROS_ERROR_STREAM_NAMED("temp","wrong size joints");

  ROS_INFO_STREAM_NAMED("temp","joint: " << joint_saturated << " Payload: " << payload);
  */

  // get payload torques
  payload = 2;
  std::vector<double> joint_torques(7, 0.0);
  if( !tester.getPayloadTorques(joint_angles, payload, joint_torques) )
    ROS_ERROR_STREAM_NAMED("temp","wrong size joints");

  std::copy(joint_torques.begin(), joint_torques.end(), std::ostream_iterator<double>(std::cout, "\n"));      

  // done
  ROS_INFO_STREAM_NAMED("trep_ros","Shutting down.");

  return 0;
}

