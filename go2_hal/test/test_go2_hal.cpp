// This file sends sinusoidal (joint position) signals to the motors. File based on unitree's position example.

#include "go2_hal/go2_hal.h"

#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <iomanip>
#include <thread>

double jointLinearInterpolation(double initPos, double targetPos, double rate)
{
		double p;
		rate = std::min(std::max(rate, 0.0), 1.0);
		p = initPos*(1-rate) + targetPos*rate;
		return p;
}

std::array<double, 3> legSinDelta(double counter)
{
	std::array<double, 3> sin_joints;

	sin_joints[0] = 0.0;
	sin_joints[1] = 0.6 * sin(3*M_PI*counter/1000.0);
	sin_joints[2] = -0.6 * sin(1.8*M_PI*counter/1000.0);

	return sin_joints;
}

void print_state(const go2hal::LowState& state)
{
		std::cout << "\33[H\33[2J" <<'\n'; //clear screen
		std::cout << std::fixed << std::setprecision(4);

		const auto& imu = state.imu_state();
		const auto& motor = state.motor_state();

		std::cout << "*---------------------------------------------*" << '\n';
		std::cout << "IMU state:" << '\n';
		std::cout << "RPY : " << imu.rpy()[0] << ", " << imu.rpy()[1] << ", " << imu.rpy()[2] << "." << '\n';
		std::cout << "Quaternion : " << imu.quaternion()[0] << ", " << imu.quaternion()[1] << ", " << imu.quaternion()[2] << ", " << imu.quaternion()[3] << "." << '\n';
		std::cout << "Gyroscope : " << imu.gyroscope()[0] << ", " << imu.gyroscope()[1] << ", " << imu.gyroscope()[2] << "." << '\n';
		std::cout << "Acceleration : " << imu.accelerometer()[0] << ", " << imu.accelerometer()[1] << ", " << imu.accelerometer()[2] << "." << '\n';

		std::cout << "*---------------------------------------------*" << '\n';
		std::cout << "Joint  state:" << '\n';
		std::cout << "LF : " << motor[go2hal::FL_0].q() << ", " << motor[go2hal::FL_1].q()	<< ", " << motor[go2hal::FL_2].q() << '\n';
		std::cout << "RF : " << motor[go2hal::FR_0].q() << ", " << motor[go2hal::FR_1].q()	<< ", " << motor[go2hal::FR_2].q()  << '\n';
		std::cout << "LH : " << motor[go2hal::RL_0].q() << ", " << motor[go2hal::RL_1].q()  << ", " << motor[go2hal::RL_2].q() << '\n';
		std::cout << "RH : " << motor[go2hal::RR_0].q() << ", " << motor[go2hal::RR_1].q()	<< ", " << motor[go2hal::RR_2].q() << '\n';

		std::cout << "*---------------------------------------------*" << '\n';
		std::cout << "Joint  velocity:" << '\n';
		std::cout << "LF : " << motor[go2hal::FL_0].dq() << ", " << motor[go2hal::FL_1].dq()  << ", " << motor[go2hal::FL_2].dq() << '\n';
		std::cout << "RF : " << motor[go2hal::FR_0].dq() << ", " << motor[go2hal::FR_1].dq()  << ", " << motor[go2hal::FR_2].dq() << '\n';
		std::cout << "LH : " << motor[go2hal::RL_0].dq() << ", " << motor[go2hal::RL_1].dq()	<< ", " << motor[go2hal::RL_2].dq() << '\n';
		std::cout << "RH : " << motor[go2hal::RR_0].dq() << ", " << motor[go2hal::RR_1].dq()  << ", " << motor[go2hal::RR_2].dq() << '\n';

		std::cout << "*---------------------------------------------*" << '\n';
		std::cout << "Joint  torque:" << '\n';
		std::cout << "LF : " << motor[go2hal::FL_0].tau_est() << ", " << motor[go2hal::FL_1].tau_est()  << ", " << motor[go2hal::FL_2].tau_est() << '\n';
		std::cout << "RF : " << motor[go2hal::FR_0].tau_est() << ", " << motor[go2hal::FR_1].tau_est()  << ", " << motor[go2hal::FR_2].tau_est() << '\n';
		std::cout << "LH : " << motor[go2hal::RL_0].tau_est() << ", " << motor[go2hal::RL_1].tau_est()	<< ", " << motor[go2hal::RL_2].tau_est() << '\n';
		std::cout << "RH : " << motor[go2hal::RR_0].tau_est() << ", " << motor[go2hal::RR_1].tau_est()  << ", " << motor[go2hal::RR_2].tau_est() << '\n';

		std::cout << std::endl;

}


int main(int argc, char** argv)
{
	if (argc < 2)
	{
		std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
		return -1;
	}

	std::cout << "Testing Go2-hal. Check the robot is suspended in the air!" << std::endl
						<< "Press Enter to continue..." << std::endl;
	std::cin.ignore();

	go2hal::LowLevelInterface robot_interface(argv[1]);

	go2hal::LowState state;
	go2hal::LowCmd lowcmd;
	robot_interface.InitCmdData(lowcmd);
	std::array<float, 60> zero_command = {0};


 	usleep(300000);
	// Send a 'dummy' command
	robot_interface.SendCommand(zero_command);

	float qInit[12] = {0};
	float qDes[12] = {0};

	int motorIdxs[12] = {
		go2hal::FL_0, go2hal::FL_1, go2hal::FL_2,
		go2hal::FR_0, go2hal::FR_1, go2hal::FR_2,
		go2hal::RL_0, go2hal::RL_1, go2hal::RL_2,
		go2hal::RR_0, go2hal::RR_1, go2hal::RR_2,
	};

	float sin_mid_q[12] = {
		0.0, 1.2, -2.0,
		0.0, 1.2, -2.0,
		0.0, 1.2, -2.0,
		0.0, 1.2, -2.0,
	};

	float ff_torques[12] = {
		+0.65, 0.0, 0.0,
		-0.65, 0.0, 0.0,
		+0.65, 0.0, 0.0,
		-0.65, 0.0, 0.0,
	};

	float Kp[12] = {0};

	float Kd[12] = {0};

	int motiontime = 0;
	int rate_count = 0;
	int sin_count = 0;
	int loop_ms = 1;

	int total_steps = 10000;

	for (int tt = 0; tt <= total_steps; ++tt)
	{
		state = robot_interface.ReceiveObservation();

		motiontime++;

		std::this_thread::sleep_for(std::chrono::milliseconds(loop_ms));

		if (motiontime >= 0 && motiontime < 1000)
		{
			for (int jj = 0; jj < 12; ++jj)
			{
				qInit[jj] = state.motor_state()[motorIdxs[jj]].q();
			}
		}

		if (motiontime >= 1000)
		{
			std::fill_n(Kp, 12, 5);
			std::fill_n(Kd, 12, 1);
		}

		if (motiontime >= 1000 && motiontime < 4000)
		{
			rate_count++;
			double rate = rate_count/3000.0;

			for (int jj = 0; jj < 12; ++jj)
			{
				qDes[jj] = jointLinearInterpolation(qInit[jj], sin_mid_q[jj], rate);
			}
		}

		if( motiontime >= 4000){
			sin_count++;

			for (int ll = 0; ll < 4; ++ll)
			{
				std::array<double, 3> sin_delta = legSinDelta(sin_count);
				for (int ii = 0; ii < 3; ++ii)
				{
					qDes[3*ll+ii] = sin_mid_q[3*ll+ii] + sin_delta[ii];
				}
			}
		}

		for (int ii = 0; ii < 12; ++ii)
		{
			auto& motor_cmd = lowcmd.motor_cmd()[motorIdxs[ii]];
			motor_cmd.mode() = 0x0A;
			motor_cmd.q()  = qDes[ii];
			motor_cmd.dq() = 0.0;
			motor_cmd.kp() = Kp[ii];
			motor_cmd.kd() = Kd[ii];
			motor_cmd.tau() = ff_torques[ii];
		}

		lowcmd.head()[0] = 0xFE;
		lowcmd.head()[1] = 0xEF;
		lowcmd.level_flag() = 0xFF; // LOWLEVEL
		robot_interface.SendLowCmd(lowcmd);
		if (tt%100 == 0)
		{
			std::cout << "Counter: " << tt << " / " << total_steps << std::endl;
			print_state(state);
		}
	}

	return 0;
}
