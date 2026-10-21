#include "go2_hal/go2_hal.h"

#include <chrono>
#include <cstring>
#include <stdexcept>
#include <thread>

namespace go2hal
{

    using namespace unitree::robot;

    LowLevelInterface::LowLevelInterface(const std::string &network_interface, int32_t domain_id)
    {
        // ChannelFactory is a process-wide singleton; Init() must be called exactly
        // once before any publisher/subscriber is created.
        std::cout << "costruttore" << std::endl;
        ChannelFactory::Instance()->Init(domain_id, network_interface);
        Init();
    }

    void LowLevelInterface::Init()
    {
        std::cout << "Init" << std::endl;

        InitLowCmd();
        low_state.imu_state().quaternion()[0] = 1.f;
        low_state.level_flag() = 0xFF; // LOWLEVEL
        /*create publisher*/
        lowcmd_publisher_.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
        lowcmd_publisher_->InitChannel();

        /*create subscriber*/
        lowstate_subscriber_.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
        lowstate_subscriber_->InitChannel(std::bind(&LowLevelInterface::LowStateMessageHandler, this, std::placeholders::_1), 1);
        std::cout << "Initialized channels" << std::endl;
        
        // deactivate sport-client
        /*init MotionSwitcherClient*/
        msc.reset(new unitree::robot::b2::MotionSwitcherClient());

        msc->SetTimeout(10.0f); 
        msc->Init();
        std::cout << "Init msc" << std::endl;

        // While the motion (sport) service is active it discards every lowcmd. Retry the release
        // until CheckMode confirms it; a failed query (-1) is retried too, since ReleaseMode is
        // harmless when nothing is running.
        const int max_release_attempts = 6;
        int service_status = queryServiceStatus();
        int32_t release_ret = -1;
        for (int attempt = 1; attempt <= max_release_attempts && service_status != 0; ++attempt)
        {
            std::cout << "Try to deactivate the motion control-related service (attempt "
                      << attempt << " of " << max_release_attempts << ")." << std::endl;
            release_ret = msc->ReleaseMode();
            if (release_ret == 0) {
                std::cout << "ReleaseMode succeeded." << std::endl;
            } else {
                std::cout << "ReleaseMode failed. Error code: " << release_ret << std::endl;
            }
            // Keep commanding while the service stops, instead of sleeping: idle motors cannot
            // be re-energised from low level.
            keepMotorsAlive(2.0);
            service_status = queryServiceStatus();
        }

        if (service_status == 0)
        {
            std::cout << "Motion control service released." << std::endl;
        }
        else if (service_status > 0)
        {
            std::cerr << "Motion control service still running after " << max_release_attempts
                      << " attempts: low-level commands will be ignored. Release it from the "
                         "remote or power-cycle the robot." << std::endl;
        }
        else if (release_ret == 0)
        {
            std::cerr << "ReleaseMode succeeded but CheckMode is not answering: could not confirm "
                         "the motion control service is released." << std::endl;
        }
        else
        {
            std::cerr << "Could neither query nor release the motion control service after "
                      << max_release_attempts << " attempts." << std::endl;
        }
        if (service_status == 0)
        {
            keepMotorsAlive(0.5);
        }

        std::cout << "finished init" << std::endl;

    }
    void LowLevelInterface::startKeepAlive()
    {
        if (keep_alive_running_.load()) return;
        keep_alive_running_.store(true);
        keep_alive_thread_ = std::thread([this]()
        {
            std::array<float, 60> zero_command = {0};
            auto next = std::chrono::steady_clock::now();
            while (keep_alive_running_.load())
            {
                SendCommand(zero_command);
                next += std::chrono::milliseconds(2);
                std::this_thread::sleep_until(next);
            }
        });
    }

    void LowLevelInterface::stopKeepAlive()
    {
        if (!keep_alive_running_.exchange(false)) return;
        if (keep_alive_thread_.joinable()) keep_alive_thread_.join();
    }

    void LowLevelInterface::keepMotorsAlive(double seconds)
    {
        std::array<float, 60> zero_command = {0};
        const auto period = std::chrono::milliseconds(2);
        const auto deadline = std::chrono::steady_clock::now()
                + std::chrono::milliseconds(static_cast<long>(seconds * 1000.0));
        auto next = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() < deadline)
        {
            SendCommand(zero_command);
            next += period;
            std::this_thread::sleep_until(next);
        }
    }

    LowLevelInterface::~LowLevelInterface()
    {
        stopKeepAlive();
    }

    void LowLevelInterface::InitLowCmd()
    {
        low_cmd.head()[0] = 0xFE;
        low_cmd.head()[1] = 0xEF;
        low_cmd.level_flag() = 0xFF;
        low_cmd.gpio() = 0;

        for (int i = 0; i < 20; i++)
        {
            low_cmd.motor_cmd()[i].mode() = (0x01); // motor switch to servo (PMSM) mode
            low_cmd.motor_cmd()[i].q() = (PosStopF);
            low_cmd.motor_cmd()[i].kp() = (0);
            low_cmd.motor_cmd()[i].dq() = (VelStopF);
            low_cmd.motor_cmd()[i].kd() = (0);
            low_cmd.motor_cmd()[i].tau() = (0);
        }
    }

    /* 1 if a motion control (sport) service is running, 0 if none is, -1 if CheckMode failed.
     * A failed CheckMode also leaves motionName empty, so it must not be read as "released". */
    int LowLevelInterface::queryServiceStatus()
    {
        std::string robotForm,motionName;
        int32_t ret = msc->CheckMode(robotForm,motionName);
        if (ret != 0)
        {
            std::cout << "CheckMode failed. Error code: " << ret << std::endl;
            return -1;
        }
        std::cout << "CheckMode succeeded." << std::endl;
        if(motionName.empty())
        {
            std::cout << "The motion control-related service is deactivated." << std::endl;
            return 0;
        }
        std::string serviceName = queryServiceName(robotForm,motionName);
        std::cout << "Service: "<< serviceName<< " is activate" << std::endl;
        return 1;
    }

    std::string LowLevelInterface::queryServiceName(std::string form,std::string name)
    {
        if(form == "0")
        {
            if(name == "normal" ) return "sport_mode"; 
            if(name == "ai" ) return "ai_sport"; 
            if(name == "advanced" ) return "advanced_sport"; 
        }
        else
        {
            if(name == "ai-w" ) return "wheeled_sport(go2W)"; 
            if(name == "normal-w" ) return "wheeled_sport(b2W)";
        }
        return "";
    }

    void LowLevelInterface::LowStateMessageHandler(const void *message)
    {
        low_state = *(unitree_go::msg::dds_::LowState_ *)message;
    }

    unitree_go::msg::dds_::LowState_ LowLevelInterface::ReceiveObservation()
    {
        return low_state;
    }

    uint32_t crc32_core(uint32_t *ptr, uint32_t len)
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

    /* The CRC is taken over the raw LowCmd_ memory, padding included, but DDS does not transmit
     * padding: the robot recomputes the CRC with the padding zeroed. A command whose padding holds
     * stale bytes (any LowCmd_ that was not explicitly zeroed, e.g. a member of a heap-allocated
     * object) therefore gets a CRC the robot rejects, every frame is dropped, and the motors fall
     * back to idle. Whether it happens depends on the heap history of the process, so the CRC is
     * computed on a copy rebuilt field by field in zeroed memory. */
    static uint32_t lowCmdCrc(const LowCmd &cmd)
    {
        LowCmd clean;
        std::memset(static_cast<void *>(&clean), 0, sizeof(clean));
        clean.head() = cmd.head();
        clean.level_flag() = cmd.level_flag();
        clean.frame_reserve() = cmd.frame_reserve();
        clean.sn() = cmd.sn();
        clean.version() = cmd.version();
        clean.bandwidth() = cmd.bandwidth();
        for (int i = 0; i < 20; ++i)
        {
            const auto &src = cmd.motor_cmd()[i];
            auto &dst = clean.motor_cmd()[i];
            dst.mode() = src.mode();
            dst.q() = src.q();
            dst.dq() = src.dq();
            dst.tau() = src.tau();
            dst.kp() = src.kp();
            dst.kd() = src.kd();
            dst.reserve() = src.reserve();
        }
        clean.bms_cmd().off() = cmd.bms_cmd().off();
        clean.bms_cmd().reserve() = cmd.bms_cmd().reserve();
        clean.wireless_remote() = cmd.wireless_remote();
        clean.led() = cmd.led();
        clean.fan() = cmd.fan();
        clean.gpio() = cmd.gpio();
        clean.reserve() = cmd.reserve();
        return crc32_core(reinterpret_cast<uint32_t *>(&clean), (sizeof(LowCmd) >> 2) - 1);
    }

    void LowLevelInterface::SendLowCmd(unitree_go::msg::dds_::LowCmd_ &cmd)
    {
        cmd.crc() = lowCmdCrc(cmd);
        lowcmd_publisher_->Write(cmd);
    }

    void LowLevelInterface::SendCommand(const std::array<float, 60> &motorcmd)
    {
        low_cmd.head()[0] = 0xFE;
        low_cmd.head()[1] = 0xEF;
        low_cmd.level_flag() = 0xFF;
        low_cmd.gpio() = 0;

        for (int motor_id = 0; motor_id < 12; ++motor_id)
        {
            auto &m = low_cmd.motor_cmd()[motor_id];

            // m.mode() = 0x01;
            // m.q() = PosStopF;
            // m.dq() = VelStopF;
            // m.kp() = motorcmd[motor_id * 5 + 2];
            // m.kd() = motorcmd[motor_id * 5 + 3];
            // m.tau() = motorcmd[motor_id * 5 + 4];
            m.mode() = 0x01;
            m.q() = motorcmd[motor_id * 5 + 0];
            m.dq() = motorcmd[motor_id * 5 + 1];
            m.kp() = motorcmd[motor_id * 5 + 2];
            m.kd() = motorcmd[motor_id * 5 + 3];
            m.tau() = motorcmd[motor_id * 5 + 4];
        }

        SendLowCmd(low_cmd);
    }

} // namespace go2hal
