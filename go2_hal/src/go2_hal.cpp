#include "go2_hal/go2_hal.h"

#include <stdexcept>

namespace go2hal
{

using namespace unitree::robot;

LowLevelInterface::LowLevelInterface(const std::string& network_interface, int32_t domain_id)
{
  // ChannelFactory is a process-wide singleton; Init() must be called exactly
  // once before any publisher/subscriber is created.
  ChannelFactory::Instance()->Init(domain_id, network_interface);

  InitCmdData(lowcmd_);

  lowstate_.imu_state().quaternion()[0] = 1.f;
  lowstate_.level_flag() = 0xFF; // LOWLEVEL

  lowcmd_publisher_.reset(new ChannelPublisher<LowCmd>(TOPIC_LOWCMD));
  lowcmd_publisher_->InitChannel();

  lowstate_subscriber_.reset(new ChannelSubscriber<LowState>(TOPIC_LOWSTATE));
  lowstate_subscriber_->InitChannel(
      std::bind(&LowLevelInterface::LowStateMessageHandler, this, std::placeholders::_1), 1);
}


bool LowLevelInterface::HasState() const
{
  return state_received_.load(std::memory_order_relaxed);
}

void LowLevelInterface::LowStateMessageHandler(const void* message)
{
  std::lock_guard<std::mutex> lock(lowstate_mutex_);
  lowstate_ = *static_cast<const LowState*>(message);
  state_received_.store(true, std::memory_order_relaxed);
}

LowState LowLevelInterface::ReceiveObservation()
{
  std::lock_guard<std::mutex> lock(lowstate_mutex_);
  return lowstate_;
}

void LowLevelInterface::InitCmdData(LowCmd& cmd)
{
  cmd.head()[0] = 0xFE;
  cmd.head()[1] = 0xEF;
  cmd.level_flag() = 0xFF; // LOWLEVEL
  cmd.gpio() = 0;

  for (auto& motor_cmd : cmd.motor_cmd())
  {
    motor_cmd.mode() = 0x0A; // motor switch to servo (PMSM) mode
    motor_cmd.q()    = PosStopF;
    motor_cmd.dq()   = VelStopF;
    motor_cmd.kp()   = 0.f;
    motor_cmd.kd()   = 0.f;
    motor_cmd.tau()  = 0.f;
  }
}

void LowLevelInterface::SendLowCmd(LowCmd& cmd)
{
  // TODO: port over safety checks equivalent to unitree_legged_sdk's
  // Safety::PositionLimit() / PowerProtect() if/when needed; unitree_sdk2
  // does not ship an equivalent helper, so any joint/power limiting must be
  // implemented here or upstream in go2_robot_hw before calling SendLowCmd().

  cmd.crc() = crc32_core(reinterpret_cast<uint32_t*>(&cmd), (sizeof(LowCmd) >> 2) - 1);
  lowcmd_publisher_->Write(cmd);
}

void LowLevelInterface::SendCommand(const std::array<float, 60>& motorcmd)
{
  lowcmd_.level_flag() = 0xFF; // LOWLEVEL
  for (int motor_id = 0; motor_id < 12; ++motor_id)
  {
    auto& m = lowcmd_.motor_cmd()[motor_id];
    m.mode() = 0x0A;
    m.q()   = motorcmd[motor_id * 5 + 0];
    m.dq()  = motorcmd[motor_id * 5 + 1];
    m.kp()  = motorcmd[motor_id * 5 + 2];
    m.kd()  = motorcmd[motor_id * 5 + 3];
    m.tau() = motorcmd[motor_id * 5 + 4];
  }
  SendLowCmd(lowcmd_);
}

} // namespace go2hal
