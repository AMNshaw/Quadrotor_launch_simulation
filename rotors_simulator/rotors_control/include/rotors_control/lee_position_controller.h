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

#ifndef ROTORS_CONTROL_LEE_POSITION_CONTROLLER_H
#define ROTORS_CONTROL_LEE_POSITION_CONTROLLER_H


#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <list>
#include <ctime>
#include <cstdlib>
#include <chrono>
#include "rotors_control/common.h"
#include "rotors_control/parameters.h"

namespace rotors_control
{

// Default values for the lee position controller and the Asctec Firefly.
static const Eigen::Vector3d kDefaultPositionGain = Eigen::Vector3d(6, 6, 6);              // 16, 16, 16
static const Eigen::Vector3d kDefaultVelocityGain = Eigen::Vector3d(4.7, 4.7, 4.7);        // 14.7, 14.7, 14.7
static const Eigen::Vector3d kDefaultAttitudeGain = Eigen::Vector3d(3, 3, 0.15);           // 2, 1.5, 0.035
static const Eigen::Vector3d kDefaultAngularRateGain = Eigen::Vector3d(0.52, 0.52, 0.18);  // 0.52, 0.52, 0.025

class LeePositionControllerParameters
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	LeePositionControllerParameters()
		: position_gain_(kDefaultPositionGain),
		  velocity_gain_(kDefaultVelocityGain),
		  attitude_gain_(kDefaultAttitudeGain),
		  angular_rate_gain_(kDefaultAngularRateGain)
	{
		calculateAllocationMatrix(rotor_configuration_, &allocation_matrix_);
	}

	Eigen::Matrix4Xd allocation_matrix_;
	Eigen::Vector3d position_gain_;
	Eigen::Vector3d velocity_gain_;
	Eigen::Vector3d attitude_gain_;
	Eigen::Vector3d angular_rate_gain_;
	RotorConfiguration rotor_configuration_;
};

class LeePositionController
{
public:
	LeePositionController();
	~LeePositionController();
	void InitializeParameters();
	void CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities, nav_msgs::Odometry* error);
	void SetOdometry(const EigenOdometry& odometry);
	void SetTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory);

	LeePositionControllerParameters controller_parameters_;
	VehicleParameters vehicle_parameters_;

	double Psi;
	Eigen::Vector3d   position_error;
	Eigen::Vector3d   velocity_error;
	Eigen::Vector3d   angular_rate_error;
	Eigen::Vector3d   angle_error;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	bool initialized_params_;
	bool controller_active_;

	/////////////// research added //////////////
	int controller_mode_;
	Eigen::Vector3d safe_position;
	/////////////////////////////////////////////

	Eigen::MatrixX4d moment_thrust_to_rotor_velocities_;

	mav_msgs::EigenTrajectoryPoint command_trajectory_;
	EigenOdometry odometry_;

	void ComputeDesiredForce(Eigen::Vector3d* force_control_input);
	void ComputeDesiredMoment(const Eigen::Vector3d& force_control_input, Eigen::Vector3d* moment_control_input);
};
}

#endif // ROTORS_CONTROL_LEE_POSITION_CONTROLLER_H
