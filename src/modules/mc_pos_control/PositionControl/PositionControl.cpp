/****************************************************************************
 *
 *   Copyright (c) 2018 - 2019 PX4 Development Team. All rights reserved.
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
 * @file PositionControl.cpp
 */

#include "PositionControl.hpp"
#include "ControlMath.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <geo/geo.h>

using namespace matrix;

PositionControl::PositionControl() {
    for (int i = 0; i < 4; ++i) {
        current_actuator_output_[i] = 0;
    }
    robot_attitude_rotation_matrix_.identity();
    _x1_vector.setZero();
    _x2_vector.setZero();
    _x3_vector.setZero();
    thrust_motor_hat_.setZero();
    thrust_motor_dot_hat_.setZero();
    thrust_motor_dot_dot_hat_.setZero();
}

void PositionControl::setVelocityGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_vel_p = P;
	_gain_vel_i = I;
	_gain_vel_d = D;
}

void PositionControl::setVelocityLimits(const float vel_horizontal, const float vel_up, const float vel_down)
{
	_lim_vel_horizontal = vel_horizontal;
	_lim_vel_up = vel_up;
	_lim_vel_down = vel_down;
}

void PositionControl::setThrustLimits(const float min, const float max)
{
	// make sure there's always enough thrust vector length to infer the attitude
	_lim_thr_min = math::max(min, 10e-4f);
	_lim_thr_max = max;
}

void PositionControl::setHorizontalThrustMargin(const float margin)
{
	_lim_thr_xy_margin = margin;
}

void PositionControl::updateHoverThrust(const float hover_thrust_new)
{
	// Given that the equation for thrust is T = a_sp * Th / g - Th
	// with a_sp = desired acceleration, Th = hover thrust and g = gravity constant,
	// we want to find the acceleration that needs to be added to the integrator in order obtain
	// the same thrust after replacing the current hover thrust by the new one.
	// T' = T => a_sp' * Th' / g - Th' = a_sp * Th / g - Th
	// so a_sp' = (a_sp - g) * Th / Th' + g
	// we can then add a_sp' - a_sp to the current integrator to absorb the effect of changing Th by Th'
	if (hover_thrust_new > FLT_EPSILON) {
		_vel_int(2) += (_acc_sp(2) - CONSTANTS_ONE_G) * _hover_thrust / hover_thrust_new + CONSTANTS_ONE_G - _acc_sp(2);
		setHoverThrust(hover_thrust_new);
	}
}

void PositionControl::setState(const PositionControlStates &states)
{
	_pos = states.position;
	_vel = states.velocity;
	_yaw = states.yaw;
	_vel_dot = states.acceleration;
}

void PositionControl::setInputSetpoint(const vehicle_local_position_setpoint_s &setpoint)
{
	_pos_sp = Vector3f(setpoint.x, setpoint.y, setpoint.z);
	_vel_sp = Vector3f(setpoint.vx, setpoint.vy, setpoint.vz);
	_acc_sp = Vector3f(setpoint.acceleration);
	_yaw_sp = setpoint.yaw;
	_yawspeed_sp = setpoint.yawspeed;
}

void PositionControl::setMaxMotorThrust(float max_motor_thrust) {
    all_motor_max_thrust_ = max_motor_thrust;
//    PX4_INFO("set all_motor_max_thrust as:%f", double(all_motor_max_thrust_));

}

void PositionControl::setPWMLimits(int32_t pwm_min_value, int32_t pwm_max_value) {
    pwm_min_value_ = pwm_min_value;
    pwm_max_value_ = pwm_max_value;
//    PX4_INFO("set pwm_min_value as:%f", double(pwm_min_value_));
//    PX4_INFO("set pwm_max_value as:%f", double(pwm_max_value_));

}

void PositionControl::setCurrentThrottle(float current_throttle) {
    current_throttle_ = current_throttle;
//    PX4_INFO("set current_throttle_ as:%f", double(current_throttle_));

}

void PositionControl::setCurrentAttitude(float attitude_quatenion[4]) {
    robot_attitude_rotation_matrix_ = matrix::Quatf(attitude_quatenion);
//    PX4_INFO("value of attitude quatenion is[x,y,z,w]:[%f,%f,%f,%f]", double(attitude_quatenion[0]), double(attitude_quatenion[1]), double(attitude_quatenion[2]), double(attitude_quatenion[3]));
}

void PositionControl::setBatteryVoltageFiltered(float battery_voltage_filtered) {
    battery_voltage_filtered_ = battery_voltage_filtered;
}

void PositionControl::setMotorValue(float motor_value[4]) {
    for (int i = 0; i < 4; ++i) {
        current_actuator_output_[i] = motor_value[i];
    }
}

bool PositionControl::update(const float dt)
{
	bool valid = _inputValid();

	if (valid) {
		_positionControl();
		_velocityControl(dt);

		_yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;
		_yaw_sp = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw; // TODO: better way to disable yaw control
	}

	// There has to be a valid output accleration and thrust setpoint otherwise something went wrong
	valid = valid && PX4_ISFINITE(_acc_sp(0)) && PX4_ISFINITE(_acc_sp(1)) && PX4_ISFINITE(_acc_sp(2));
	valid = valid && PX4_ISFINITE(_thr_sp(0)) && PX4_ISFINITE(_thr_sp(1)) && PX4_ISFINITE(_thr_sp(2));

	return valid;
}

bool PositionControl::updateWithDisturbanceRejection(const float dt) {
    bool valid = _inputValid();

    if (valid) {
        _positionControl();
        _velocityControl(dt);

        _yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;
        _yaw_sp = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw; // TODO: better way to disable yaw control
    }

    // There has to be a valid output accleration and thrust setpoint otherwise something went wrong
    valid = valid && PX4_ISFINITE(_acc_sp(0)) && PX4_ISFINITE(_acc_sp(1)) && PX4_ISFINITE(_acc_sp(2));
    valid = valid && PX4_ISFINITE(_thr_sp(0)) && PX4_ISFINITE(_thr_sp(1)) && PX4_ISFINITE(_thr_sp(2));

    return valid;
}

void PositionControl::_positionControl()
{
	// P-position controller
	Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p);

	// Position and feed-forward velocity setpoints or position states being NAN results in them not having an influence
	ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);
	// make sure there are no NAN elements for further reference while constraining
	ControlMath::setZeroIfNanVector3f(vel_sp_position);

	// Constrain horizontal velocity by prioritizing the velocity component along the
	// the desired position setpoint over the feed-forward term.
	_vel_sp.xy() = ControlMath::constrainXY(vel_sp_position.xy(), (_vel_sp - vel_sp_position).xy(), _lim_vel_horizontal);
	// Constrain velocity in z-direction.
	_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);
}

void PositionControl::_velocityControl(const float dt)
{
	// PID velocity control
	Vector3f vel_error = _vel_sp - _vel;
	Vector3f acc_sp_velocity = vel_error.emult(_gain_vel_p) + _vel_int - _vel_dot.emult(_gain_vel_d);

	// No control input from setpoints or corresponding states which are NAN
	ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);

    _accelerationControl(dt);

	// Integrator anti-windup in vertical direction
	if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.0f) ||
	    (_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.0f)) {
		vel_error(2) = 0.f;
	}

	// Prioritize vertical control while keeping a horizontal margin
	const Vector2f thrust_sp_xy(_thr_sp);
	const float thrust_sp_xy_norm = thrust_sp_xy.norm();
	const float thrust_max_squared = math::sq(_lim_thr_max);

	// Determine how much vertical thrust is left keeping horizontal margin
	const float allocated_horizontal_thrust = math::min(thrust_sp_xy_norm, _lim_thr_xy_margin);
	const float thrust_z_max_squared = thrust_max_squared - math::sq(allocated_horizontal_thrust);

	// Saturate maximal vertical thrust
	_thr_sp(2) = math::max(_thr_sp(2), -sqrtf(thrust_z_max_squared));

	// Determine how much horizontal thrust is left after prioritizing vertical control
	const float thrust_max_xy_squared = thrust_max_squared - math::sq(_thr_sp(2));
	float thrust_max_xy = 0;

	if (thrust_max_xy_squared > 0) {
		thrust_max_xy = sqrtf(thrust_max_xy_squared);
	}

	// Saturate thrust in horizontal direction
	if (thrust_sp_xy_norm > thrust_max_xy) {
		_thr_sp.xy() = thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy;
	}

	// Use tracking Anti-Windup for horizontal direction: during saturation, the integrator is used to unsaturate the output
	// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
	const Vector2f acc_sp_xy_limited = Vector2f(_thr_sp) * (CONSTANTS_ONE_G / _hover_thrust);
	const float arw_gain = 2.f / _gain_vel_p(0);
	vel_error.xy() = Vector2f(vel_error) - (arw_gain * (Vector2f(_acc_sp) - acc_sp_xy_limited));

	// Make sure integral doesn't get NAN
	ControlMath::setZeroIfNanVector3f(vel_error);
	// Update integral part of velocity control
	_vel_int += vel_error.emult(_gain_vel_i) * dt;

	// limit thrust integral
	_vel_int(2) = math::min(fabsf(_vel_int(2)), CONSTANTS_ONE_G) * sign(_vel_int(2));
}

void PositionControl::_accelerationControl(const float dt)
{
	// Assume standard acceleration due to gravity in vertical direction for attitude generation
	Vector3f body_z = Vector3f(-_acc_sp(0), -_acc_sp(1), CONSTANTS_ONE_G).normalized();
	ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);
	// Scale thrust assuming hover thrust produces standard gravity
	float collective_thrust = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
	// Project thrust to planned body attitude
	collective_thrust /= (Vector3f(0, 0, 1).dot(body_z));
	collective_thrust = math::min(collective_thrust, -_lim_thr_min);
	_thr_sp = body_z * collective_thrust;
    // Update thrust with disturbance rejection
//    PX4_INFO("current position z is:%f",double(_pos(2)));
    PX4_INFO("desired_throttle is:%f", double(_thr_sp(2)));
    if (_ndrc_pos_enable){
        calculateDisturbanceRejectionThrust(_thr_sp, dt);
    }
}

void
PositionControl::calculateDisturbanceRejectionThrust(matrix::Vector3f &thr_sp, const float dt) {
    estimatorUpdateThreeOrder(_pos(2), dt, _x1, _x2, _x3);
    float thrust_hat = _mass * (_x3 + CONSTANTS_ONE_G);
    float throttle_hat = thrustToThrottle(thrust_hat);
    estimatorUpdateThreeOrder(current_throttle_, dt, throttle_motor_hat_, throttle_motor_dot_hat_,
                              throttle_motor_dot_dot_hat_);
    throttle_disturbance_ = throttle_hat - throttle_motor_hat_;
    PX4_INFO("current_throttle is:%f", double(current_throttle_));
    PX4_INFO("thrust_hat is:%f", double(thrust_hat));
    PX4_INFO("throttle_hat is:%f", double(throttle_hat));
    PX4_INFO("throttle_motor_hat_ is:%f", double(throttle_motor_hat_));
    PX4_INFO("throttle_disturbance_ = throttle_hat - throttle_motor_hat_ is:%f", double(throttle_disturbance_));
    _thr_sp(2) = _thr_sp(2) - throttle_disturbance_;
}

void PositionControl::calculateDisturbanceRejectionThrustTest(matrix::Vector3f &thr_sp, const float dt) {
    matrix::Vector3f gravity(0.0f, 0.0f, _mass * CONSTANTS_ONE_G);
    estimatorUpdateThreeOrderVector(_pos, dt, _x1_vector, _x2_vector, _x3_vector);
    matrix::Vector3f thrust_hat = -_mass * _x3_vector + gravity;
    calculateOneMotorMaxThrust();
    matrix::Vector3f actual_thrust_from_motor = calculateActualThrustFromMotor();
//    float thrust_hat_normalized = thrustToThrottle(thrust_hat);
    estimatorUpdateThreeOrderVector(actual_thrust_from_motor, dt, thrust_motor_hat_, thrust_motor_dot_hat_,
                                    thrust_motor_dot_dot_hat_);
    matrix::Vector3f thrust_disturbance = thrust_hat - thrust_motor_hat_;
    PX4_INFO("current_thrust is:%f", double(actual_thrust_from_motor(2)));
    PX4_INFO("thrust_disturbance = thrust_hat - thrust_motor_hat is:%f", double(thrust_disturbance(2)));
    setMaxMotorThrust(4*one_motor_max_thrust_);

    for (int i = 0; i < 3; ++i) {
        _thr_sp(i) = thr_sp(i) + thrustToThrottle(thrust_disturbance(i));
    }
//    float throttle_disturbance_compensation = thrustToThrottle(thrust_disturbance);
////    float thrust_compensation = math::constrain(thrust_disturbance, -CONSTANTS_ONE_G, CONSTANTS_ONE_G);
//    _thr_sp(2) = thr_sp(2) + throttle_disturbance_compensation;
}

float PositionControl::thrustToNormalizedInput(float &thrust) {
    setMaxMotorThrust(4.0f * one_motor_max_thrust_);
//    PX4_INFO("all_motor_max_thrust is:%f", double(all_motor_max_thrust_));
//    PX4_INFO("thrust_disturbance_normalized is:%f", double(thrust/all_motor_max_thrust_));
//    PX4_INFO("==========================================================");
    return thrust / all_motor_max_thrust_;
}

matrix::Vector3f PositionControl::calculateActualThrustFromMotor() {
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
//    throttle(i) = current_throttle_;
    }
    thrust = throttle * one_motor_max_thrust_;
    float thrust_total = thrust(0) + thrust(1) + thrust(2) + thrust(3);

    matrix::Vector3f thrust_body(0.0f, 0.0f, thrust_total);//TODO check the minus - here.
//	if(_num_update%200 == 0){
//    PX4_INFO("thrust_max = %8.6f", (double) one_motor_max_thrust_);
//    PX4_INFO("pwm = %8.6f, %8.6f, %8.6f, %8.6f", (double) pwm[0], (double) pwm[1], (double) pwm[2], (double) pwm[3]);
//    PX4_INFO("throttle = %8.6f, %8.6f, %8.6f, %8.6f", (double) throttle(0), (double) throttle(1), (double) throttle(2),
//             (double) throttle(3));
//    PX4_INFO("thrust = %8.6f, %8.6f, %8.6f, %8.6f", (double) thrust(0), (double) thrust(1), (double) thrust(2),
//             (double) thrust(3));
//	}
    return robot_attitude_rotation_matrix_ * thrust_body;
}

void PositionControl::calculateOneMotorMaxThrust() {//TODO check the calculation of max motor thrust.
    if(battery_voltage_filtered_ < 11.1f){
        battery_voltage_filtered_ = 11.1f;
    } else if(battery_voltage_filtered_ > 12.6f){
        battery_voltage_filtered_ = 12.6;
    }
    one_motor_max_thrust_ = 1.0f * 5.488f * sinf(battery_voltage_filtered_ * 0.4502f + 2.2241f);
//    one_motor_max_thrust_ = 5.488f;
}

float PositionControl::thrustToThrottle(float actual_thrust) {
    return actual_thrust / all_motor_max_thrust_;
}

void PositionControl::estimateUpdate(float x, const float dt, float &x1,
                                     float &x2) {
    float x1_dot_1, x2_dot_1, x1_dot_2, x2_dot_2, x1_dot_3, x2_dot_3, x1_dot_4, x2_dot_4;
    float x1_tmp, x2_tmp;

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

void PositionControl::estimatorUpdateThreeOrder(float x, const float dt, float &x1, float &x2, float &x3) {
    float x1_dot_1, x2_dot_1, x3_dot_1, x1_dot_2, x2_dot_2, x3_dot_2, x1_dot_3, x2_dot_3, x3_dot_3, x1_dot_4, x2_dot_4, x3_dot_4;
    float x1_tmp, x2_tmp, x3_tmp;
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

void PositionControl::estimatorUpdateThreeOrderVector(matrix::Vector3f x, const float dt, matrix::Vector3f &x1, matrix::Vector3f &x2, matrix::Vector3f &x3) {
    matrix::Vector3f x1_dot_1, x2_dot_1, x3_dot_1, x1_dot_2, x2_dot_2, x3_dot_2, x1_dot_3, x2_dot_3, x3_dot_3, x1_dot_4, x2_dot_4, x3_dot_4;
    matrix::Vector3f x1_tmp, x2_tmp, x3_tmp;
    x1_tmp = x1;	// x1: estimate of x
    x2_tmp = x2;	// x2: estimate of x's acceleration
    x3_tmp = x3;
    estimateModelThreeOrderVector(x, x1_tmp, x2_tmp, x3_tmp, x1_dot_1, x2_dot_1, x3_dot_1);

    x1_tmp = x1 + x1_dot_1 * dt/2.0f;
    x2_tmp = x2 + x2_dot_1 * dt/2.0f;
    x3_tmp = x3 + x3_dot_1 * dt/2.0f;
    estimateModelThreeOrderVector(x, x1_tmp, x2_tmp, x3_tmp, x1_dot_2, x2_dot_2, x3_dot_2);

    x1_tmp = x1 + x1_dot_2 * dt/2.0f;
    x2_tmp = x2 + x2_dot_2 * dt/2.0f;
    x3_tmp = x3 + x3_dot_2 * dt/2.0f;
    estimateModelThreeOrderVector(x, x1_tmp, x2_tmp, x3_tmp, x1_dot_3, x2_dot_3, x3_dot_3);

    x1_tmp = x1 + x1_dot_3 * dt;
    x2_tmp = x2 + x2_dot_3 * dt;
    x3_tmp = x3 + x3_dot_3 * dt;
    estimateModelThreeOrderVector(x, x1_tmp, x2_tmp, x3_tmp, x1_dot_4, x2_dot_4, x3_dot_4);

    x1 += (x1_dot_1 + x1_dot_2 * 2.0f + x1_dot_3 * 2.0f + x1_dot_4) * dt/6.0f;
    x2 += (x2_dot_1 + x2_dot_2 * 2.0f + x2_dot_3 * 2.0f + x2_dot_4) * dt/6.0f;
    x3 += (x3_dot_1 + x3_dot_2 * 2.0f + x3_dot_3 * 2.0f + x3_dot_4) * dt/6.0f;
}

void PositionControl::estimateModelThreeOrder(float &pos, float &x1, float &x2, float &x3, float &x1_dot, float &x2_dot,
                                              float &x3_dot) {
    x1_dot = x2;
    x2_dot = x3;
    x3_dot = (pos - x1) * _omega_pos2 * _omega_pos2 * _omega_pos1
             - x2 * (_omega_pos2 * _omega_pos2 + 2.0f * _zeta_pos * _omega_pos1 * _omega_pos2)
             - x3 * (2.0f * _zeta_pos * _omega_pos2 + _omega_pos1);
}

void PositionControl::estimateModelThreeOrderVector(matrix::Vector3f &pos, matrix::Vector3f &x1, matrix::Vector3f &x2,
                                                    matrix::Vector3f &x3, matrix::Vector3f &x1_dot,
                                                    matrix::Vector3f &x2_dot,
                                                    matrix::Vector3f &x3_dot) {
    x1_dot = x2;
    x2_dot = x3;
    x3_dot = (pos - x1) * _omega_pos2 * _omega_pos2 * _omega_pos1
             - x2 * (_omega_pos2 * _omega_pos2 + 2.0f * _zeta_pos * _omega_pos1 * _omega_pos2)
             - x3 * (2.0f * _zeta_pos * _omega_pos2 + _omega_pos1);
}

void PositionControl::estimatorModel(float &rates, float &x1, float &x2,
                                     float &x1_dot, float &x2_dot) {
    x1_dot = x2;
    x2_dot = (rates - x1) * _omega_att * _omega_att - x2 * 2.0f * _zeta_att * _omega_att;
}

bool PositionControl::_inputValid()
{
	bool valid = true;

	// Every axis x, y, z needs to have some setpoint
	for (int i = 0; i <= 2; i++) {
		valid = valid && (PX4_ISFINITE(_pos_sp(i)) || PX4_ISFINITE(_vel_sp(i)) || PX4_ISFINITE(_acc_sp(i)));
	}

	// x and y input setpoints always have to come in pairs
	valid = valid && (PX4_ISFINITE(_pos_sp(0)) == PX4_ISFINITE(_pos_sp(1)));
	valid = valid && (PX4_ISFINITE(_vel_sp(0)) == PX4_ISFINITE(_vel_sp(1)));
	valid = valid && (PX4_ISFINITE(_acc_sp(0)) == PX4_ISFINITE(_acc_sp(1)));

	// For each controlled state the estimate has to be valid
	for (int i = 0; i <= 2; i++) {
		if (PX4_ISFINITE(_pos_sp(i))) {
			valid = valid && PX4_ISFINITE(_pos(i));
		}

		if (PX4_ISFINITE(_vel_sp(i))) {
			valid = valid && PX4_ISFINITE(_vel(i)) && PX4_ISFINITE(_vel_dot(i));
		}
	}

	return valid;
}

void PositionControl::getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const
{
	local_position_setpoint.x = _pos_sp(0);
	local_position_setpoint.y = _pos_sp(1);
	local_position_setpoint.z = _pos_sp(2);
	local_position_setpoint.yaw = _yaw_sp;
	local_position_setpoint.yawspeed = _yawspeed_sp;
	local_position_setpoint.vx = _vel_sp(0);
	local_position_setpoint.vy = _vel_sp(1);
	local_position_setpoint.vz = _vel_sp(2);
	_acc_sp.copyTo(local_position_setpoint.acceleration);
	_thr_sp.copyTo(local_position_setpoint.thrust);
}

void PositionControl::getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const
{
	ControlMath::thrustToAttitude(_thr_sp, _yaw_sp, attitude_setpoint);
	attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp;
}
