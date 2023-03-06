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
 * @file RateControl.cpp
 */

#include <RateControl.hpp>
#include <px4_platform_common/defines.h>

using namespace matrix;
RateControl::RateControl() {
    rate_hat_.setZero();
    rate_dot_hat_.setZero();
    rate_dot_dot_hat_.setZero();
    current_actuator_command_hat_.setZero();
    current_actuator_command_dot_hat_.setZero();
    current_actuator_command_dot_dot_hat_.setZero();
}

void RateControl::setPWMLimits(int32_t pwm_min_value, int32_t pwm_max_value) {
    pwm_min_value_ = pwm_min_value;
    pwm_max_value_ = pwm_max_value;

}

void RateControl::setCurrentAttitude(float attitude_quatenion[4]) {
    robot_attitude_rotation_matrix_ = matrix::Quatf(attitude_quatenion);
}

void RateControl::setBatteryVoltageFiltered(float battery_voltage_filtered) {
    battery_voltage_filtered_ = battery_voltage_filtered;
}

void RateControl::setMotorValue(float motor_value[4]) {
    for (int i = 0; i < 4; ++i) {
        current_actuator_output_[i] = motor_value[i];
    }
}

matrix::Vector3f RateControl::calculateActualTorqueFromMotor() {
    float array_torque[3][4] = {
            {-_half_length, _half_length, _half_length, -_half_length},
            {_half_width, -_half_width, _half_width, -_half_width},
            {_C_M, _C_M, -_C_M, -_C_M}
    };

    Matrix<float, 3, 4> gentrq_matrix(array_torque);

    // motor thrust model:
    // thrust = (1 - _thrust_factor) * PWM + _thrust_factor * PWM^2
    float pwm[4];
    float pwm_length = pwm_max_value_ - pwm_min_value_;
    for (int i = 0; i < 4; ++i) {
        pwm[i] = math::constrain((current_actuator_output_[i] - pwm_min_value_) / pwm_length, 0.f, 1.f);
    }
    matrix::Vector<float, 4> throttle, thrust;
    for (int i = 0; i < 4; ++i) {
        throttle(i) = (1.0f - thrust_factor_) * pwm[i] + thrust_factor_ * pwm[i] * pwm[i];
    }
    thrust = throttle * all_motor_max_thrust_;

    return gentrq_matrix * thrust;
}

void RateControl::calculateOneMotorMaxThrust() {//TODO check the calculation of max motor thrust.
    if(battery_voltage_filtered_ < 11.1f){
        battery_voltage_filtered_ = 11.1f;
    } else if(battery_voltage_filtered_ > 12.6f){
        battery_voltage_filtered_ = 12.6;
    }
    one_motor_max_thrust_ = 1.0f * 5.488f * sinf(battery_voltage_filtered_ * 0.4502f + 2.2241f);
//    one_motor_max_thrust_ = 5.488f;
}

void RateControl::setMaxMotorThrust(float max_motor_thrust) {
    all_motor_max_thrust_ = max_motor_thrust;
//    PX4_INFO("set all_motor_max_thrust as:%f", double(all_motor_max_thrust_));
}


void RateControl::setGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_p = P;
	_gain_i = I;
	_gain_d = D;
}

void RateControl::setSaturationStatus(const Vector<bool, 3> &saturation_positive,
				      const Vector<bool, 3> &saturation_negative)
{
	_control_allocator_saturation_positive = saturation_positive;
	_control_allocator_saturation_negative = saturation_negative;
}

Vector3f RateControl::update(const Vector3f &rate, const Vector3f &rate_sp, const Vector3f &angular_accel,
			     const float dt, const bool landed)
{
	// angular rates error
	Vector3f rate_error = rate_sp - rate;

	// PID control with feed forward
	const Vector3f torque = _gain_p.emult(rate_error) + _rate_int - _gain_d.emult(angular_accel) + _gain_ff.emult(rate_sp);

	// update integral only if we are not landed
	if (!landed) {
		updateIntegral(rate_error, dt);
	}

	return torque;
}

Vector3f RateControl::updateDisturbanceRejectionTorques(const matrix::Vector3f &rate, const matrix::Vector3f &rate_sp,
                                                        const matrix::Vector3f &angular_accel, const float dt,
                                                        const bool landed, const matrix::Vector3f &current_actuator_command, float current_position_z) {
    matrix::Vector3f torque_hat;    // estimate of torque
    matrix::Vector3f torque_disturbance;
//    estimateUpdate(rate, dt, rate_hat_, rate_dot_hat_);
    estimatorUpdateThreeOrder(rate,dt,rate_hat_,rate_dot_hat_,rate_dot_dot_hat_);
    matrix::Vector3f inertia(_I_XX, _I_YY, _I_ZZ);
//    PX4_INFO("rate are: %f, %f, %f",double(rate(0)),double(rate(1)),double(rate(2)));
//    PX4_INFO("rate_hat after filter are: %f, %f, %f",double(rate_hat_(0)),double(rate_hat_(1)),double(rate_hat_(2)));

    torque_hat = inertia.emult(rate_dot_hat_) + rate_hat_.cross(inertia.emult(rate_hat_));  // tau = I*w_dot + w x I*w

    // angular rates error
    Vector3f rate_error = rate_sp - rate;
//    estimateUpdate(current_actuator_command, dt, current_actuator_command_hat_, current_actuator_command_dot_hat_);
    estimatorUpdateThreeOrder(current_actuator_command,dt,current_actuator_command_hat_,current_actuator_command_dot_hat_,current_actuator_command_dot_dot_hat_);
    torque_disturbance = torque_hat - current_actuator_command_hat_;
//    PX4_INFO("torque_motor are: %f, %f, %f", double(current_actuator_command(0)), double(current_actuator_command(1)), double(current_actuator_command(2)));
//    PX4_INFO("torque_motor_hat are: %f, %f, %f", double(current_actuator_command_hat_(0)), double(current_actuator_command_hat_(1)), double(current_actuator_command_hat_(2)));
//    PX4_INFO("torque_hat are: %f, %f, %f", double(torque_hat(0)), double(torque_hat(1)), double(torque_hat(2)));
//    PX4_INFO("torque_disturbances are: %f, %f, %f", double(torque_disturbance(0)), double(torque_disturbance(1)), double(torque_disturbance(2)));

    // PID control with feed forward
    const Vector3f torque_pid = _gain_p.emult(rate_error) + _rate_int - _gain_d.emult(angular_accel) + _gain_ff.emult(rate_sp);
    if (!_ndrc_att_enable || current_position_z<=0.3f){
        // update integral only if we are not landed
        if (!landed) {
            updateIntegral(rate_error, dt);
        }
        return torque_pid;
    } else{
        Vector3f torque_model_compensation = inertia.emult(rate_dot_hat_) + rate.cross(inertia.emult(rate));
        Vector3f torque_pid_model_compensation = torque_pid + torque_model_compensation;
        Vector3f torque_disturbance_rejection = torque_pid_model_compensation - torque_disturbance;
        if (!landed) {
            updateIntegral(rate_error, dt);
        }
//            PX4_INFO("torque_disturbance_rejection are:%f, %f, %f", double(torque_disturbance_rejection(0)),
//                     double(torque_disturbance_rejection(1)), double(torque_disturbance_rejection(2)));
//            PX4_INFO("================================================");
        return torque_disturbance_rejection;
    }

}

Vector3f
RateControl::updateDisturbanceRejectionTorquesTest(const matrix::Vector3f &rate, const matrix::Vector3f &rate_sp,
                                                   const matrix::Vector3f &angular_accel, const float dt,
                                                   const bool landed, const matrix::Vector3f &current_actuator_command,
                                                   float current_position_z) {
    matrix::Vector3f torque_hat;    // estimate of torque
    matrix::Vector3f torque_disturbance;
//    estimateUpdate(rate, dt, rate_hat_, rate_dot_hat_);
    estimatorUpdateThreeOrder(rate, dt, rate_hat_, rate_dot_hat_, rate_dot_dot_hat_);
    matrix::Vector3f inertia(_I_XX, _I_YY, _I_ZZ);
//    PX4_INFO("rate are: %f, %f, %f",double(rate(0)),double(rate(1)),double(rate(2)));
//    PX4_INFO("rate_hat after filter are: %f, %f, %f",double(rate_hat_(0)),double(rate_hat_(1)),double(rate_hat_(2)));
    calculateOneMotorMaxThrust();
    setMaxMotorThrust(4*one_motor_max_thrust_);
    current_actuator_command_ = calculateActualTorqueFromMotor();

    torque_hat = inertia.emult(rate_dot_hat_) + rate_hat_.cross(inertia.emult(rate_hat_));  // tau = I*w_dot + w x I*w

    // angular rates error
    Vector3f rate_error = rate_sp - rate;
//    estimateUpdate(current_actuator_command, dt, current_actuator_command_hat_, current_actuator_command_dot_hat_);
    estimatorUpdateThreeOrder(current_actuator_command_, dt, current_actuator_command_hat_,
                              current_actuator_command_dot_hat_, current_actuator_command_dot_dot_hat_);
    torque_disturbance = torque_hat - current_actuator_command_hat_;
//    PX4_INFO("torque_motor are: %f, %f, %f", double(current_actuator_command(0)), double(current_actuator_command(1)), double(current_actuator_command(2)));
//    PX4_INFO("torque_motor_hat are: %f, %f, %f", double(current_actuator_command_hat_(0)), double(current_actuator_command_hat_(1)), double(current_actuator_command_hat_(2)));
//    PX4_INFO("torque_hat are: %f, %f, %f", double(torque_hat(0)), double(torque_hat(1)), double(torque_hat(2)));
//    PX4_INFO("torque_disturbances are: %f, %f, %f", double(torque_disturbance(0)), double(torque_disturbance(1)), double(torque_disturbance(2)));

    // PID control with feed forward
    const Vector3f torque_pid =
            _gain_p.emult(rate_error) + _rate_int - _gain_d.emult(angular_accel) + _gain_ff.emult(rate_sp);
    if (!_ndrc_att_enable || current_position_z <= 0.5f) {
        // update integral only if we are not landed
        if (!landed) {
            updateIntegral(rate_error, dt);
        }
        return torque_pid;
    } else {
        Vector3f torque_model_compensation = inertia.emult(rate_dot_hat_) + rate.cross(inertia.emult(rate));
        Vector3f torque_pid_model_compensation = torque_pid + torque_model_compensation;
        Vector3f torque_disturbance_rejection = torque_pid_model_compensation - torque_disturbance;
        if (!landed) {
            updateIntegral(rate_error, dt);
        }
//            PX4_INFO("torque_disturbance_rejection are:%f, %f, %f", double(torque_disturbance_rejection(0)),
//                     double(torque_disturbance_rejection(1)), double(torque_disturbance_rejection(2)));
//            PX4_INFO("================================================");
        return torque_disturbance_rejection;
    }

}

void RateControl::estimatorUpdateThreeOrder(matrix::Vector3f x, const float dt, matrix::Vector3f &x1,
                                            matrix::Vector3f &x2, matrix::Vector3f &x3) {
    matrix::Vector3f x1_dot_1, x2_dot_1, x3_dot_1, x1_dot_2, x2_dot_2, x3_dot_2, x1_dot_3, x2_dot_3, x3_dot_3, x1_dot_4, x2_dot_4, x3_dot_4;
    matrix::Vector3f x1_tmp, x2_tmp, x3_tmp;
    x1_tmp = x1;	// x1: estimate of x
    x2_tmp = x2;	// x2: estimate of x's acceleration
    x3_tmp = x3;
    estimateModelThreeOrder(x, x1_tmp, x2_tmp, x3_tmp, x1_dot_1, x2_dot_1, x3_dot_1);

    x1_tmp = x1 + x1_dot_1 * dt/2.0f;
    x2_tmp = x2 + x2_dot_1 * dt/2.0f;
    x3_tmp = x3 + x3_dot_1 * dt/2.0f;
    estimateModelThreeOrder(x, x1_tmp, x2_tmp, x3_tmp, x1_dot_2, x2_dot_2, x3_dot_2);

    x1_tmp = x1 + x1_dot_2 * dt/2.0f;
    x2_tmp = x2 + x2_dot_2 * dt/2.0f;
    x3_tmp = x3 + x3_dot_2 * dt/2.0f;
    estimateModelThreeOrder(x, x1_tmp, x2_tmp, x3_tmp, x1_dot_3, x2_dot_3, x3_dot_3);

    x1_tmp = x1 + x1_dot_3 * dt;
    x2_tmp = x2 + x2_dot_3 * dt;
    x3_tmp = x3 + x3_dot_3 * dt;
    estimateModelThreeOrder(x, x1_tmp, x2_tmp, x3_tmp, x1_dot_4, x2_dot_4, x3_dot_4);

    x1 += (x1_dot_1 + x1_dot_2 * 2.0f + x1_dot_3 * 2.0f + x1_dot_4) * dt/6.0f;
    x2 += (x2_dot_1 + x2_dot_2 * 2.0f + x2_dot_3 * 2.0f + x2_dot_4) * dt/6.0f;
    x3 += (x3_dot_1 + x3_dot_2 * 2.0f + x3_dot_3 * 2.0f + x3_dot_4) * dt/6.0f;
}

void RateControl::estimateModelThreeOrder(matrix::Vector3f &pos, matrix::Vector3f &x1, matrix::Vector3f &x2,
                                          matrix::Vector3f &x3, matrix::Vector3f &x1_dot, matrix::Vector3f &x2_dot,
                                          matrix::Vector3f &x3_dot) {
    x1_dot = x2;
    x2_dot = x3;
    x3_dot = (pos - x1) * _omega_pos2 * _omega_pos2 * _omega_pos1
             - x2 * (_omega_pos2 * _omega_pos2 + 2.0f * _zeta_pos * _omega_pos1 * _omega_pos2)
             - x3 * (2.0f * _zeta_pos * _omega_pos2 + _omega_pos1);
}

void RateControl::estimateUpdate(matrix::Vector3f x, const float dt, matrix::Vector3f &x1,
                                 matrix::Vector3f &x2) {
    Vector3f x1_dot_1, x2_dot_1, x1_dot_2, x2_dot_2, x1_dot_3, x2_dot_3, x1_dot_4, x2_dot_4;
    Vector3f x1_tmp, x2_tmp;

    x1_tmp = x1;	// x1: estimate of x
    x2_tmp = x2;	// x2: estimate of x' acceleration
    estimatorModel(x, x1_tmp, x2_tmp, x1_dot_1, x2_dot_1);

    x1_tmp = x1 + x1_dot_1 * dt/2.0f;
    x2_tmp = x2 + x2_dot_1 * dt/2.0f;
    estimatorModel(x, x1_tmp, x2_tmp, x1_dot_2, x2_dot_2);

    x1_tmp = x1 + x1_dot_2 * dt/2.0f;
    x2_tmp = x2 + x2_dot_2 * dt/2.0f;
    estimatorModel(x, x1_tmp, x2_tmp, x1_dot_3, x2_dot_3);

    x1_tmp = x1 + x1_dot_3 * dt;
    x2_tmp = x2 + x2_dot_3 * dt;
    estimatorModel(x, x1_tmp, x2_tmp, x1_dot_4, x2_dot_4);

    x1 += (x1_dot_1 + x1_dot_2 * 2.0f + x1_dot_3 * 2.0f + x1_dot_4) * dt/6.0f;
    x2 += (x2_dot_1 + x2_dot_2 * 2.0f + x2_dot_3 * 2.0f + x2_dot_4) * dt/6.0f;
}

void RateControl::estimatorModel(matrix::Vector3f &rates, matrix::Vector3f &x1, matrix::Vector3f &x2,
                                 matrix::Vector3f &x1_dot, matrix::Vector3f &x2_dot) {
    x1_dot = x2;
    x2_dot = (rates - x1) * _omega_att * _omega_att - x2 * 2.0f * _zeta_att * _omega_att;
}

void RateControl::updateIntegral(Vector3f &rate_error, const float dt)
{
	for (int i = 0; i < 3; i++) {
		// prevent further positive control saturation
		if (_control_allocator_saturation_positive(i)) {
			rate_error(i) = math::min(rate_error(i), 0.f);
		}

		// prevent further negative control saturation
		if (_control_allocator_saturation_negative(i)) {
			rate_error(i) = math::max(rate_error(i), 0.f);
		}

		// I term factor: reduce the I gain with increasing rate error.
		// This counteracts a non-linear effect where the integral builds up quickly upon a large setpoint
		// change (noticeable in a bounce-back effect after a flip).
		// The formula leads to a gradual decrease w/o steps, while only affecting the cases where it should:
		// with the parameter set to 400 degrees, up to 100 deg rate error, i_factor is almost 1 (having no effect),
		// and up to 200 deg error leads to <25% reduction of I.
		float i_factor = rate_error(i) / math::radians(400.f);
		i_factor = math::max(0.0f, 1.f - i_factor * i_factor);

		// Perform the integration using a first order method
		float rate_i = _rate_int(i) + i_factor * _gain_i(i) * rate_error(i) * dt;

		// do not propagate the result if out of range or invalid
		if (PX4_ISFINITE(rate_i)) {
			_rate_int(i) = math::constrain(rate_i, -_lim_int(i), _lim_int(i));
		}
	}
}

void RateControl::getRateControlStatus(rate_ctrl_status_s &rate_ctrl_status)
{
	rate_ctrl_status.rollspeed_integ = _rate_int(0);
	rate_ctrl_status.pitchspeed_integ = _rate_int(1);
	rate_ctrl_status.yawspeed_integ = _rate_int(2);
}
