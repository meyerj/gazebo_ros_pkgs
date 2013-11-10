//=================================================================================================
// Copyright (c) 2013, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef _GAZEBO_ROS_CONTROL___WRENCH_ROBOT_HW_SIM_H_
#define _GAZEBO_ROS_CONTROL___WRENCH_ROBOT_HW_SIM_H_

// ros_control
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/hardware_interface.h>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

// gazebo_ros_control
#include <gazebo_ros_control/robot_hw_sim.h>

namespace gazebo_ros_control
{

class WrenchRobotHWSim : public gazebo_ros_control::RobotHWSim, public hardware_interface::HardwareInterface
{
public:

  bool initSim(
    const std::string& robot_namespace,
    ros::NodeHandle model_nh,
    gazebo::physics::ModelPtr parent_model,
    const urdf::Model *const urdf_model,
    std::vector<transmission_interface::TransmissionInfo> transmissions);

  void readSim(ros::Time time, ros::Duration period);

  void writeSim(ros::Time time, ros::Duration period);

  const gazebo::math::Pose &getPose()
  {
    return pose_;
  }

  void setWrench(const gazebo::math::Vector3 &force, const gazebo::math::Vector3 &torque = gazebo::math::Vector3::Zero)
  {
    force_ = force;
    torque_ = torque;
  }

private:
  gazebo::physics::ModelPtr parent_model_;

  gazebo::math::Pose pose_;
  gazebo::math::Vector3 force_;
  gazebo::math::Vector3 torque_;
};

}

#endif // #ifndef _GAZEBO_ROS_CONTROL___WRENCH_ROBOT_HW_SIM_H_
