/*
 * Copyright (C) 2022 Francesco Roscia
 * Author: Francesco Roscia
 * email:  francesco.roscia@iit.it
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#ifndef GO2_ROBOT_HW_H
#define GO2_ROBOT_HW_H

#include <base_hardware_interface/base_robot_hw.h>
#include <go2_hal/go2_hal.h>

#include <memory>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <realtime_tools/realtime_publisher.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>

#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>

typedef Eigen::Matrix<double, 6,1> butterFilterParams;


namespace go22ros
{

class Go2RobotHw : public hardware_interface::RobotHW, public hardware_interface::WolfRobotHwInterface
{
public:
  Go2RobotHw();
  virtual ~Go2RobotHw();

  void init();
  void read();
  void write();

private:

  /**
   * @brief Map WoLF joint order (LF,LH,RF,RH) to Go2 internal motor indices.
   *
   * NOTE: with unitree_sdk2 the 20-slot motor array is ordered
   * FR, FL, RR, RL (hip, thigh, calf) -- see go2hal::JointIndex --
   * which is *different* from the FL,RL,FR,RR ordering used by the
   * legacy unitree_legged_sdk-based HAL.
   */
  std::array<unsigned int, 12> go2_motor_idxs_
          {{
          go2hal::FL_0, go2hal::FL_1, go2hal::FL_2, // LF
          go2hal::RL_0, go2hal::RL_1, go2hal::RL_2, // LH
          go2hal::FR_0, go2hal::FR_1, go2hal::FR_2, // RF
          go2hal::RR_0, go2hal::RR_1, go2hal::RR_2, // RH
          }};

  /** @brief Go2-HAL (unitree_sdk2 / DDS based). Constructed in init() once the
   *  network interface name has been read from the ROS parameter server. */
  std::unique_ptr<go2hal::LowLevelInterface> go2_interface_;
  go2hal::LowState go2_state_;
  go2hal::LowCmd go2_lowcmd_;

  /** @brief Sends a zero command to the robot */
  void send_zero_command();

  /** @brief Executes the robot's startup routine */
  void startup_routine();


  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>> contact_state_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::Imu>> imu_pub_;

  /** @brief IMU realtime publisher */
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odom_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Vector3>> imu_acc_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Vector3>> imu_euler_pub_;
  std::vector<butterFilterParams> velocityFilterBuffer;
  void filt(const double raw, butterFilterParams & filt);

  std::vector<double> imu_euler_raw_;
  std::vector<double> imu_orientation_raw_;
  std::vector<double> remove_euler_;
  std::vector<double> remove_quaternion_;
  bool is_remove_yaw_set_ = false;
  unsigned int base_pub_counter = 0;

};

}

#endif
