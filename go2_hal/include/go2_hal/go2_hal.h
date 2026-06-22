#ifndef _GO2_HAL_H_
#define _GO2_HAL_H_

/*
 * go2_hal — Hardware Abstraction Layer for the Unitree Go2, built on top of
 * unitree_sdk2 (DDS / CycloneDDS channels), as opposed to the older
 * unitree_legged_sdk (raw UDP) used for Go1/Aliengo.
 *
 * The public API intentionally mirrors the previous go2_hal that wrapped
 * unitree_legged_sdk (LowLevelInterface::ReceiveObservation / SendLowCmd /
 * SendCommand / InitCmdData) so that go2_robot_hw only needs to change *how*
 * it reads/writes individual fields (accessor methods instead of public
 * members), not the overall control flow.
 *
 * Based on third_party/unitree_sdk2/example/go2/go2_low_level.cpp
 */

#include <array>
#include <atomic>
#include <mutex>
#include <string>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>

#include <unitree/dds_wrapper/common/crc.h>
#include <unitree/robot/go2/robot_state/robot_state_client.hpp>

#include <iostream>
#include <thread>
#include <chrono>

namespace go2hal
{

typedef  unitree_go::msg::dds_::LowState_ LowState;

typedef  unitree_go::msg::dds_::LowCmd_ LowCmd;

/** @brief DDS topic names used for the low-level control interface. */
constexpr const char* TOPIC_LOWCMD   = "rt/lowcmd";
constexpr const char* TOPIC_LOWSTATE = "rt/lowstate";

/**
 * @brief Index of each leg motor inside the 20-slot motor_cmd()/motor_state()
 * arrays of LowCmd_/LowState_.
 *
 * NOTE: this ordering (FR, FL, RR, RL - Hip, Thigh, Calf) follows
 * unitree::robot::go2::JointIndex from unitree_sdk2 and is *different* from
 * the FL,RL,FR,RR ordering used by the legacy unitree_legged_sdk-based HAL.
 * Any joint-index mapping kept from the Go1/legacy go2_robot_hw must be
 * updated accordingly.
 */
enum JointIndex
{
  FR_0 = 0, FR_1 = 1, FR_2 = 2,   // FR hip, thigh, calf
  FL_0 = 3, FL_1 = 4, FL_2 = 5,   // FL hip, thigh, calf
  RR_0 = 6, RR_1 = 7, RR_2 = 8,   // RR hip, thigh, calf
  RL_0 = 9, RL_1 = 10, RL_2 = 11, // RL hip, thigh, calf
};

/** @brief "Don't care" sentinel values for position/velocity, used when commanding pure torque. */
constexpr float PosStopF = 2.146E+9f;
constexpr float VelStopF = 16000.0f;

/**
 * @brief Low-level (joint torque/position/velocity) control interface for the Go2.
 *
 * Wraps a DDS publisher on "rt/lowcmd" and a DDS subscriber on "rt/lowstate",
 * following the same pattern as the official go2_low_level.cpp example.
 */
class LowLevelInterface
{
public:



/**
   * @param network_interface Network interface connected to the robot (e.g. "eth0").
   *        Forwarded to unitree::robot::ChannelFactory::Instance()->Init().
   * @param domain_id DDS domain id, 0 unless your setup requires otherwise.
   */
  explicit LowLevelInterface(const std::string& network_interface, int32_t domain_id = 0);

  ~LowLevelInterface() = default;

  LowLevelInterface(const LowLevelInterface&) = delete;
  LowLevelInterface& operator=(const LowLevelInterface&) = delete;

  /** @brief Returns the most recently received LowState_ (thread-safe). */
  unitree_go::msg::dds_::LowState_ ReceiveObservation();

  /**
   * @brief Initializes a LowCmd_ with safe defaults: every motor in PMSM
   * (servo) mode, zero torque, position/velocity targets set to "don't care".
   */
  void InitLowCmd();

  /** @brief Computes the CRC and publishes the command over DDS. */
  void SendLowCmd(unitree_go::msg::dds_::LowCmd_& cmd);

  /**
   * @brief Convenience overload matching the legacy go2_hal API: builds a
   * LowCmd_ from a flat array of 12 x [q, dq, Kp, Kd, tau] and sends it.
   * IMPORTANT: this ensures Kp/Kd are explicitly set (e.g. to zero) every call.
   */
  void SendCommand(const std::array<float, 60>& motorcmd);
  bool HasState() const;


  private:

    void Init();
    void LowStateMessageHandler(const void* messages);
 
    std::array<unsigned int, 12> go2_motor_idxs_;

 
    unitree_go::msg::dds_::LowCmd_ low_cmd{};
    unitree_go::msg::dds_::LowState_ low_state{};

    unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_>    lowcmd_publisher_;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_ > lowstate_subscriber_;

    /*LowCmd write thread*/
 
    std::thread lowCmdWriteThread_;
};

} // namespace go2hal

#endif // _GO2_HAL_H_
