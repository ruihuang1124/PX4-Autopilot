/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file RateControl.hpp
 *
 * PID 3 axis angular rate / angular velocity control.
 */

#pragma once

#include <matrix/matrix/math.hpp>

#include <lib/mixer/MultirotorMixer/MultirotorMixer.hpp>
#include <uORB/topics/rate_ctrl_status.h>

class RateControl
{
public:
	RateControl();
	~RateControl() = default;


    void setPWMLimits(int32_t pwm_min_value, int32_t pwm_max_value);

    void setBatteryVoltageFiltered(float battery_voltage_filtered);

    void setMotorValue(float motor_value[4]);

    void setCurrentAttitude(float attitude_quatenion[4]);
    void setMaxMotorThrust(float max_motor_thrust);
	/**
	 * Set the rate control gains
	 * @param P 3D vector of proportional gains for body x,y,z axis
	 * @param I 3D vector of integral gains
	 * @param D 3D vector of derivative gains
	 */
	void setGains(const matrix::Vector3f &P, const matrix::Vector3f &I, const matrix::Vector3f &D);

	/**
	 * Set the mximum absolute value of the integrator for all axes
	 * @param integrator_limit limit value for all axes x, y, z
	 */
	void setIntegratorLimit(const matrix::Vector3f &integrator_limit) { _lim_int = integrator_limit; };

	/**
	 * Set direct rate to torque feed forward gain
	 * @see _gain_ff
	 * @param FF 3D vector of feed forward gains for body x,y,z axis
	 */
	void setFeedForwardGain(const matrix::Vector3f &FF) { _gain_ff = FF; };

	/**
	 * Set saturation status
	 * @param control saturation vector from control allocator
	 */
	void setSaturationStatus(const matrix::Vector<bool, 3> &saturation_positive,
				 const matrix::Vector<bool, 3> &saturation_negative);

	/**
	 * Run one control loop cycle calculation
	 * @param rate estimation of the current vehicle angular rate
	 * @param rate_sp desired vehicle angular rate setpoint
	 * @param dt desired vehicle angular rate setpoint
	 * @return [-1,1] normalized torque vector to apply to the vehicle
	 */
	matrix::Vector3f update(const matrix::Vector3f &rate, const matrix::Vector3f &rate_sp,
				const matrix::Vector3f &angular_accel, const float dt, const bool landed);

    /**
     *
     * @param rate estimation of the current vehicle angular rate
     * @param rate_sp desired vehicle angular rate setpoint
     * @param angular_accel
     * @param dt desired vehicle angular rate setpoint
     * @param landed
     * @return [-1,1] normalized torque vector to apply to the vehicle
     */
    matrix::Vector3f updateDisturbanceRejectionTorques(const matrix::Vector3f &rate, const matrix::Vector3f &rate_sp,
                                                       const matrix::Vector3f &angular_accel, const float dt, const bool landed, const matrix::Vector3f &current_actuator_command, float current_position_z);

    matrix::Vector3f updateDisturbanceRejectionTorquesTest(const matrix::Vector3f &rate, const matrix::Vector3f &rate_sp,
                                                           const matrix::Vector3f &angular_accel, const float dt, const bool landed, const matrix::Vector3f &current_actuator_command, float current_position_z);

    /**
     *
     * @param x
     * @param dt
     * @param x1
     * @param x2
     */

    void estimateUpdate(matrix::Vector3f x, const float dt, matrix::Vector3f &x1, matrix::Vector3f &x2);

    void estimatorModel(matrix::Vector3f &rates, matrix::Vector3f &x1, matrix::Vector3f &x2, matrix::Vector3f &x1_dot, matrix::Vector3f &x2_dot);

    void estimatorUpdateThreeOrder(matrix::Vector3f x, const float dt, matrix::Vector3f &x1, matrix::Vector3f &x2, matrix::Vector3f &x3);

    void estimateModelThreeOrder(matrix::Vector3f &pos, matrix::Vector3f &x1, matrix::Vector3f &x2, matrix::Vector3f &x3, matrix::Vector3f &x1_dot, matrix::Vector3f &x2_dot, matrix::Vector3f &x3_dot);

	/**
	 * Set the integral term to 0 to prevent windup
	 * @see _rate_int
	 */
	void resetIntegral() { _rate_int.zero(); }

	/**
	 * Get status message of controller for logging/debugging
	 * @param rate_ctrl_status status message to fill with internal states
	 */
	void getRateControlStatus(rate_ctrl_status_s &rate_ctrl_status);

private:
	void updateIntegral(matrix::Vector3f &rate_error, const float dt);

    matrix::Vector3f calculateActualTorqueFromMotor();
    void calculateOneMotorMaxThrust();

	// Gains
	matrix::Vector3f _gain_p; ///< rate control proportional gain for all axes x, y, z
	matrix::Vector3f _gain_i; ///< rate control integral gain
	matrix::Vector3f _gain_d; ///< rate control derivative gain
	matrix::Vector3f _lim_int; ///< integrator term maximum absolute value
	matrix::Vector3f _gain_ff; ///< direct rate to torque feed forward gain only useful for helicopters

	// States
	matrix::Vector3f _rate_int; ///< integral term of the rate controller

	// Feedback from control allocation
	matrix::Vector<bool, 3> _control_allocator_saturation_negative;
	matrix::Vector<bool, 3> _control_allocator_saturation_positive;
    float 	_omega_att = 80.0f;
    float	_zeta_att = 0.8f;
    float	_I_XX	= 0.019125f;
    float	_I_YY	= 0.019125f;
    float	_I_ZZ	= 0.045225f;
    matrix::Vector3f rate_hat_, rate_dot_hat_, rate_dot_dot_hat_;
    matrix::Vector3f current_actuator_command_hat_, current_actuator_command_dot_hat_, current_actuator_command_dot_dot_hat_;
    bool _ndrc_att_enable{true};
    float	_omega_pos1 = 100.0f;
    float	_omega_pos2 = 80.0f;
    float	_zeta_pos  = 0.8f;

    int32_t pwm_min_value_=0, pwm_max_value_=0;
    float current_actuator_output_[4];
    matrix::Dcmf robot_attitude_rotation_matrix_;
    float battery_voltage_filtered_=0.0f;
    matrix::Vector3f current_actuator_command_;
    float all_motor_max_thrust_{100.0};
    float one_motor_max_thrust_ = 0.0f;
    float	_half_length = 0.101f;
    float	_half_width	 = 0.08f;
    float	_C_M	= 	0.0097f;
    float thrust_factor_ = 0.4f;
};
