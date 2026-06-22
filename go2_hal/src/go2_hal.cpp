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

  unitree::robot::go2::RobotStateClient rsc;
  // before creating/sending lowcmd, release sport/internal motion mode
  rsc.SetTimeout(10.0f);
  rsc.Init();

  int32_t status = 0;
  int32_t ret = rsc.ServiceSwitch("sport_mode", 0, status);
  std::this_thread::sleep_for(std::chrono::seconds(1));
  ret = rsc.ServiceSwitch("sport_mode", 0, status);
  std::this_thread::sleep_for(std::chrono::seconds(1)); 
  ret = rsc.ServiceSwitch("sport_mode", 0, status);
  std::this_thread::sleep_for(std::chrono::seconds(1));  
  std::cout << "[Go2 HAL] ServiceSwitch sport_mode OFF ret: " << ret << ", status: " << status << std::endl;

  // Create publishers and subscribers to talk with Unitree
  Init();

}

void LowLevelInterface::Init()
{
    InitLowCmd();
    low_state.imu_state().quaternion()[0] = 1.f;
    low_state.level_flag() = 0xFF; // LOWLEVEL
    /*create publisher*/
    lowcmd_publisher_.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_publisher_->InitChannel();

    /*create subscriber*/
    lowstate_subscriber_.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber_->InitChannel(std::bind(&LowLevelInterface::LowStateMessageHandler, this, std::placeholders::_1), 1);

    /*loop publishing thread*/
    //lowCmdWriteThread_ =    std::thread(&LowLevelInterface::SendCommand, this);
}

void LowLevelInterface::InitLowCmd()
{
    low_cmd.head()[0] = 0xFE;
    low_cmd.head()[1] = 0xEF;
    low_cmd.level_flag() = 0xFF;
    low_cmd.gpio() = 0;

    for(int i=0; i<20; i++)
    {
        low_cmd.motor_cmd()[i].mode() = (0x01);   // motor switch to servo (PMSM) mode
        low_cmd.motor_cmd()[i].q() = (PosStopF);
        low_cmd.motor_cmd()[i].kp() = (0);
        low_cmd.motor_cmd()[i].dq() = (VelStopF);
        low_cmd.motor_cmd()[i].kd() = (0);
        low_cmd.motor_cmd()[i].tau() = (0);
    }
}
 

void LowLevelInterface::LowStateMessageHandler(const void* message)
{
    low_state = *(unitree_go::msg::dds_::LowState_*)message;
}


unitree_go::msg::dds_::LowState_ LowLevelInterface::ReceiveObservation()
{
  return low_state;
}



uint32_t crc32_core(uint32_t* ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }

    return CRC32;
}

void LowLevelInterface::SendLowCmd(unitree_go::msg::dds_::LowCmd_& cmd)
{
   cmd.crc() = crc32_core(reinterpret_cast<uint32_t*>(&cmd), (sizeof(unitree_go::msg::dds_::LowCmd_ ) >> 2) - 1);
   lowcmd_publisher_->Write(cmd);
 }

void LowLevelInterface::SendCommand(const std::array<float, 60>& motorcmd)
{
  low_cmd.head()[0] = 0xFE;
  low_cmd.head()[1] = 0xEF;
  low_cmd.level_flag() = 0xFF;
  low_cmd.gpio() = 0;

  for (int motor_id = 0; motor_id < 12; ++motor_id)
  {
    auto& m = low_cmd.motor_cmd()[motor_id];

    m.mode() = 0x0A;
    m.q()    = PosStopF;
    m.dq()   = VelStopF;
    m.kp()   = motorcmd[motor_id * 5 + 2];
    m.kd()   = motorcmd[motor_id * 5 + 3];
    m.tau()  = motorcmd[motor_id * 5 + 4];
  }

  SendLowCmd(low_cmd);
}

} // namespace go2hal
