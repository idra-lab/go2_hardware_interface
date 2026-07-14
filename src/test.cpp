// go2_hal_stand_demo.cpp
#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <array>
#include <chrono>
#include <thread>
#include <atomic>
#include <unistd.h>

#include "go2_hal/go2_hal.h"

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

class Go2HALStandDemo
{
public:
    explicit Go2HALStandDemo(const std::string &network_interface, int32_t domain_id = 0)
        : hal_interface_(network_interface, domain_id)
    {}

    ~Go2HALStandDemo() = default;

    void Init();
    void Start();

private:
    void InitLowCmd();
    void LowCmdWrite();
    void PrintSensorData();

private:
    go2hal::LowLevelInterface hal_interface_;

    float Kp = 60.0;
    float Kd = 5.0;
    double time_consume = 0;
    int rate_count = 0;
    int sin_count = 0;
    int motiontime = 0;
    float dt = 0.002; // 0.001~0.01

    std::array<float, 60> motor_cmd_{};

    float _targetPos_1[12] = {0.0, 1.36, -2.65, 0.0, 1.36, -2.65,
                              -0.2, 1.36, -2.65, 0.2, 1.36, -2.65};

    float _targetPos_2[12] = {0.0, 0.67, -1.3, 0.0, 0.67, -1.3,
                              0.0, 0.67, -1.3, 0.0, 0.67, -1.3};

    float _targetPos_3[12] = {-0.35, 1.36, -2.65, 0.35, 1.36, -2.65,
                              -0.5, 1.36, -2.65, 0.5, 1.36, -2.65};

    float _startPos[12];
    float _duration_1 = 500;   
    float _duration_2 = 500; 
    float _duration_3 = 1000;   
    float _duration_4 = 900;   
    float _percent_1 = 0;    
    float _percent_2 = 0;    
    float _percent_3 = 0;    
    float _percent_4 = 0;    

    bool firstRun = true;
    bool done = false;
    bool printSensorData = false;
    std::atomic<bool> running_{true};
};

void Go2HALStandDemo::Init()
{
    InitLowCmd();
    
    // LowLevelInterface already initializes DDS and disables sport_mode
    // No need to call MotionSwitcherClient as go2_hal handles this

    std::cout << "[Go2HAL Stand Demo] Initialized" << std::endl;
    std::cout << "WARNING: Make sure the robot is hung up or lying on the ground." << std::endl;
}

void Go2HALStandDemo::InitLowCmd()
{
    // Initialize motor command array with safe defaults
    for (int motor_id = 0; motor_id < 12; ++motor_id)
    {
        motor_cmd_[motor_id * 5 + 0] = PosStopF;  // q
        motor_cmd_[motor_id * 5 + 1] = VelStopF;  // dq
        motor_cmd_[motor_id * 5 + 2] = 0;         // Kp
        motor_cmd_[motor_id * 5 + 3] = 0;         // Kd
        motor_cmd_[motor_id * 5 + 4] = 0;         // tau
    }
}

void Go2HALStandDemo::PrintSensorData()
{
    if (_percent_4 < 1)
    {
        auto low_state = hal_interface_.ReceiveObservation();
        std::cout << "Read sensor data example: " << std::endl;
        std::cout << "Joint 0 pos: " << low_state.motor_state()[0].q() << std::endl;
        std::cout << "Imu accelerometer : " << "x: " << low_state.imu_state().accelerometer()[0] 
                  << " y: " << low_state.imu_state().accelerometer()[1] 
                  << " z: " << low_state.imu_state().accelerometer()[2] << std::endl;
        std::cout << "Foot force " << low_state.foot_force()[0] << std::endl;
        std::cout << std::endl;
    }
    if ((_percent_4 == 1) && (done == false))
    {
        std::cout << "The example is done! " << std::endl;
        std::cout << std::endl;
        done = true;
    }
}

void Go2HALStandDemo::LowCmdWrite()
{
    PrintSensorData();

    motiontime++;
    if (motiontime >= 500)
    {
        if (firstRun)
        {
            auto low_state = hal_interface_.ReceiveObservation();
            for (int i = 0; i < 12; i++)
            {
                _startPos[i] = low_state.motor_state()[i].q();
            }
            firstRun = false;
        }

        _percent_1 += (float)1 / _duration_1;
        _percent_1 = _percent_1 > 1 ? 1 : _percent_1;
        if (_percent_1 < 1)
        {
            for (int j = 0; j < 12; j++)
            {
                motor_cmd_[j * 5 + 0] = (1 - _percent_1) * _startPos[j] + _percent_1 * _targetPos_1[j];
                motor_cmd_[j * 5 + 1] = 0;
                motor_cmd_[j * 5 + 2] = Kp;
                motor_cmd_[j * 5 + 3] = Kd;
                motor_cmd_[j * 5 + 4] = 0;
            }
        }
        if ((_percent_1 == 1) && (_percent_2 < 1))
        {
            _percent_2 += (float)1 / _duration_2;
            _percent_2 = _percent_2 > 1 ? 1 : _percent_2;

            for (int j = 0; j < 12; j++)
            {
                motor_cmd_[j * 5 + 0] = (1 - _percent_2) * _targetPos_1[j] + _percent_2 * _targetPos_2[j];
                motor_cmd_[j * 5 + 1] = 0;
                motor_cmd_[j * 5 + 2] = Kp;
                motor_cmd_[j * 5 + 3] = Kd;
                motor_cmd_[j * 5 + 4] = 0;
            }
        }

        if ((_percent_1 == 1) && (_percent_2 == 1) && (_percent_3 < 1))
        {
            _percent_3 += (float)1 / _duration_3;
            _percent_3 = _percent_3 > 1 ? 1 : _percent_3;

            for (int j = 0; j < 12; j++)
            {
                motor_cmd_[j * 5 + 0] = _targetPos_2[j];
                motor_cmd_[j * 5 + 1] = 0;
                motor_cmd_[j * 5 + 2] = Kp;
                motor_cmd_[j * 5 + 3] = Kd;
                motor_cmd_[j * 5 + 4] = 0;
            }
        }
        if ((_percent_1 == 1) && (_percent_2 == 1) && (_percent_3 == 1) && ((_percent_4 <= 1)))
        {
            _percent_4 += (float)1 / _duration_4;
            _percent_4 = _percent_4 > 1 ? 1 : _percent_4;
            for (int j = 0; j < 12; j++)
            {
                motor_cmd_[j * 5 + 0] = (1 - _percent_4) * _targetPos_2[j] + _percent_4 * _targetPos_3[j];
                motor_cmd_[j * 5 + 1] = 0;
                motor_cmd_[j * 5 + 2] = Kp;
                motor_cmd_[j * 5 + 3] = Kd;
                motor_cmd_[j * 5 + 4] = 0;
            }
        }

        // Send the command using HAL interface
        hal_interface_.SendCommand(motor_cmd_);
    }
}

void Go2HALStandDemo::Start()
{
    // Main loop - runs at approximately 500Hz (2ms period)
    const auto period = std::chrono::milliseconds(2);
    auto next_time = std::chrono::steady_clock::now();

    while (running_)
    {
        LowCmdWrite();
        next_time += period;
        std::this_thread::sleep_until(next_time);
    }
}

int main(int argc, const char** argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface [domain_id]" << std::endl;
        std::cout << "Example: " << argv[0] << " eth0" << std::endl;
        std::cout << "Example: " << argv[0] << " eth0 0" << std::endl;
        exit(-1);
    }

    std::string network_interface = argv[1];
    int32_t domain_id = 0;
    if (argc >= 3)
    {
        domain_id = std::stoi(argv[2]);
    }

    std::cout << "WARNING: Make sure the robot is hung up or lying on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    std::cout << "[Go2HAL Stand Demo] Starting with interface: " << network_interface 
              << ", domain_id: " << domain_id << std::endl;

    Go2HALStandDemo demo(network_interface, domain_id);
    demo.Init();
    demo.Start();

    return 0;
}