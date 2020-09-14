/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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

#include "RoverAttitudeControl.hpp"

#include <vtol_att_control/vtol_type.h>

using namespace time_literals;
using math::constrain;
using math::gradual;
using math::radians;

RoverAttitudeControl::RoverAttitudeControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::attitude_ctrl),
	_actuators_0_pub(ORB_ID(actuator_controls_0)),
	_attitude_sp_pub(ORB_ID(vehicle_attitude_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	/* fetch initial parameter values */
	parameters_update();

	// set initial maximum body rate setpoints
	_yaw_ctrl.set_max_rate(radians(_param_rov_acro_yaw_max.get()));
}

RoverAttitudeControl::~RoverAttitudeControl()
{
	perf_free(_loop_perf);
}

bool
RoverAttitudeControl::init()
{
	if (!_att_sub.registerCallback()) {
		PX4_ERR("vehicle attitude callback registration failed!");
		return false;
	}

	return true;
}

int
RoverAttitudeControl::parameters_update()
{
	updateParams();
	/* yaw control parameters */
	_yaw_ctrl.set_k_p(_param_rov_yr_p.get());
	_yaw_ctrl.set_k_i(_param_rov_yr_i.get());
	_yaw_ctrl.set_k_ff(_param_rov_yr_ff.get());
	_yaw_ctrl.set_integrator_max(_param_rov_yr_imax.get());

	return PX4_OK;
}

void
RoverAttitudeControl::vehicle_control_mode_poll()
{
	_vcontrol_mode_sub.update(&_vcontrol_mode);
}

void
RoverAttitudeControl::vehicle_manual_poll()
{
	if (_manual_control_setpoint_sub.update(&_manual_control_setpoint)) {
		if (_vcontrol_mode.flag_control_manual_enabled) {
			if (_vcontrol_mode.flag_control_attitude_enabled) {
				// STABILIZED mode generate the attitude setpoint from manual user inputs
				_att_sp.roll_body = 0;
				_att_sp.pitch_body = 0;
				_att_sp.yaw_body = _manual_control_setpoint.r;
				_att_sp.thrust_body[0] = _manual_control_setpoint.z;

				Quatf q(Eulerf(_att_sp.roll_body, _att_sp.pitch_body, _att_sp.yaw_body));
				q.copyTo(_att_sp.q_d);

				_att_sp.timestamp = hrt_absolute_time();

				_attitude_sp_pub.publish(_att_sp);

			} else if (_vcontrol_mode.flag_control_rates_enabled) {

				// RATE mode we need to generate the rate setpoint from manual user inputs
				_rates_sp.timestamp = hrt_absolute_time();
				_rates_sp.roll = 0;
				_rates_sp.pitch = 0;
				_rates_sp.yaw = _manual_control_setpoint.r * radians(_param_rov_acro_yaw_max.get());
				_rates_sp.thrust_body[0] = _manual_control_setpoint.z;

				_rate_sp_pub.publish(_rates_sp);

			} else {
				// manual/direct control
				_actuators.control[actuator_controls_s::INDEX_ROLL] = 0;
				_actuators.control[actuator_controls_s::INDEX_PITCH] = 0;
				_actuators.control[actuator_controls_s::INDEX_YAW] =
					_manual_control_setpoint.r * _param_rov_man_y_sc.get() + _param_trim_yaw.get();
				_actuators.control[actuator_controls_s::INDEX_THROTTLE] = _manual_control_setpoint.z;

				_actuators_0_pub.publish(_actuators);
			}
		}
	}
}

void
RoverAttitudeControl::vehicle_attitude_setpoint_poll()
{
	if (_att_sp_sub.update(&_att_sp)) {
		_rates_sp.thrust_body[0] = _att_sp.thrust_body[0];
		_rates_sp.thrust_body[1] = _att_sp.thrust_body[1];
		_rates_sp.thrust_body[2] = _att_sp.thrust_body[2];
	}
}

void
RoverAttitudeControl::vehicle_rates_setpoint_poll()
{
	if (_rates_sp_sub.update(&_rates_sp)) {

	}
}

void RoverAttitudeControl::Run()
{
	if (should_exit()) {
		_att_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// only run controller if attitude changed
	vehicle_attitude_s att;

	if (_att_sub.update(&att)) {

		// only update parameters if they changed
		bool params_updated = _parameter_update_sub.updated();

		// check for parameter updates
		if (params_updated) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			parameters_update();
		}

		const float dt = math::constrain((att.timestamp - _last_run) * 1e-6f, 0.002f, 0.04f);
		_last_run = att.timestamp;

		/* get current rotation matrix and euler angles from control state quaternions */
		matrix::Dcmf R = matrix::Quatf(att.q);

		vehicle_angular_velocity_s angular_velocity{};
		_vehicle_rates_sub.copy(&angular_velocity);
		float yawspeed = angular_velocity.xyz[2];

		const matrix::Eulerf euler_angles(R);

		vehicle_manual_poll();
		vehicle_attitude_setpoint_poll();	// must be updated after vehicle_manual_poll(), to get latest sp

		// vehicle status update must be before the vehicle_control_mode_poll(), otherwise rate sp are not published during whole transition
		_vehicle_status_sub.update(&_vehicle_status);
		vehicle_control_mode_poll();

		_local_pos_sub.update(&_local_pos);

		/* lock integrator until control is started or for long intervals (> 20 ms) */
		bool lock_integrator = !_vcontrol_mode.flag_control_rates_enabled
				       || (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROVER)
				       || (dt > 0.02f);

		/* decide if in stabilized or full manual control */
		if (_vcontrol_mode.flag_control_rates_enabled) {

			/* reset integrals where needed */
			if (_att_sp.yaw_reset_integral) {
				_yaw_ctrl.reset_integrator();
			}

			/* Prepare data for attitude controllers */
			ECL_ControlData control_input{};
			control_input.roll = euler_angles.phi();
			control_input.pitch = euler_angles.theta();
			control_input.yaw = euler_angles.psi();
			control_input.body_z_rate = yawspeed;
			control_input.roll_setpoint = _att_sp.roll_body;
			control_input.pitch_setpoint = _att_sp.pitch_body;
			control_input.yaw_setpoint = _att_sp.yaw_body;
			control_input.lock_integrator = lock_integrator;

			/* reset body angular rate limits on mode change */
			if ((_vcontrol_mode.flag_control_attitude_enabled != _flag_control_attitude_enabled_last) || params_updated) {
				if (_vcontrol_mode.flag_control_attitude_enabled
				    || _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
					_yaw_ctrl.set_max_rate(radians(_param_rov_y_rmax.get()));

				} else {
					_yaw_ctrl.set_max_rate(radians(_param_rov_acro_yaw_max.get()));
				}
			}

			_flag_control_attitude_enabled_last = _vcontrol_mode.flag_control_attitude_enabled;

			/* bi-linear interpolation over airspeed for actuator trim scheduling */
			float trim_yaw = _param_trim_yaw.get();

			/* Run attitude controllers */
			if (_vcontrol_mode.flag_control_attitude_enabled) {
				if (PX4_ISFINITE(_att_sp.roll_body) && PX4_ISFINITE(_att_sp.pitch_body)) {

					_yaw_ctrl.control_attitude(dt, control_input);


					/* Update input data for rate controllers */
					control_input.roll_rate_setpoint = 0;//_roll_ctrl.get_desired_rate();
					control_input.pitch_rate_setpoint = 0;//_pitch_ctrl.get_desired_rate();
					control_input.yaw_rate_setpoint = _yaw_ctrl.get_desired_rate();

					/* Run attitude RATE controllers which need the desired attitudes from above, add trim */

					float yaw_u = 0.0f;

					yaw_u = _yaw_ctrl.control_euler_rate(dt, control_input);


					_actuators.control[actuator_controls_s::INDEX_YAW] = (PX4_ISFINITE(yaw_u)) ? yaw_u + trim_yaw : trim_yaw;

					/* add in manual rudder control in manual modes */
					if (_vcontrol_mode.flag_control_manual_enabled) {
						_actuators.control[actuator_controls_s::INDEX_YAW] += _manual_control_setpoint.r;
					}

					if (!PX4_ISFINITE(yaw_u)) {
						_yaw_ctrl.reset_integrator();
					}

					/* throttle passed through if it is finite and if no engine failure was detected */
					_actuators.control[actuator_controls_s::INDEX_THROTTLE] = (PX4_ISFINITE(_att_sp.thrust_body[0])
							&& !_vehicle_status.engine_failure) ? _att_sp.thrust_body[0] : 0.0f;

					/* scale effort by battery status */
					if (_param_rov_bat_scale_en.get() &&
					    _actuators.control[actuator_controls_s::INDEX_THROTTLE] > 0.1f) {

						if (_battery_status_sub.updated()) {
							battery_status_s battery_status{};

							if (_battery_status_sub.copy(&battery_status)) {
								if (battery_status.scale > 0.0f) {
									_battery_scale = battery_status.scale;
								}
							}
						}

						_actuators.control[actuator_controls_s::INDEX_THROTTLE] *= _battery_scale;
					}
				}

				/*
				 * Lazily publish the rate setpoint (for analysis, the actuators are published below)
				 * only once available
				 */
				_rates_sp.yaw = _yaw_ctrl.get_desired_bodyrate();

				_rates_sp.timestamp = hrt_absolute_time();

				_rate_sp_pub.publish(_rates_sp);

			} else {
				vehicle_rates_setpoint_poll();

				_yaw_ctrl.set_bodyrate_setpoint(_rates_sp.yaw);

				float yaw_u = _yaw_ctrl.control_bodyrate(dt, control_input);
				_actuators.control[actuator_controls_s::INDEX_YAW] = (PX4_ISFINITE(yaw_u)) ? yaw_u + trim_yaw : trim_yaw;

				_actuators.control[actuator_controls_s::INDEX_THROTTLE] = PX4_ISFINITE(_rates_sp.thrust_body[0]) ?
						_rates_sp.thrust_body[0] : 0.0f;
			}

			rate_ctrl_status_s rate_ctrl_status{};
			rate_ctrl_status.timestamp = hrt_absolute_time();

			rate_ctrl_status.yawspeed_integ = _yaw_ctrl.get_integrator();


			_rate_ctrl_status_pub.publish(rate_ctrl_status);
		}

		_actuators.control[5] = _manual_control_setpoint.aux1;
		// FIXME: this should use _vcontrol_mode.landing_gear_pos in the future
		_actuators.control[7] = _manual_control_setpoint.aux3;

		/* lazily publish the setpoint only once available */
		_actuators.timestamp = hrt_absolute_time();
		_actuators.timestamp_sample = att.timestamp;

		/* Only publish if any of the proper modes are enabled */
		if (_vcontrol_mode.flag_control_rates_enabled ||
		    _vcontrol_mode.flag_control_attitude_enabled ||
		    _vcontrol_mode.flag_control_manual_enabled) {
			_actuators_0_pub.publish(_actuators);
		}
	}

	perf_end(_loop_perf);
}

int RoverAttitudeControl::task_spawn(int argc, char *argv[])
{
	RoverAttitudeControl *instance = new RoverAttitudeControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int RoverAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int RoverAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
rover_att_control is the rover/boat attitude controller.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rover_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int rover_att_control_main(int argc, char *argv[])
{
	return RoverAttitudeControl::main(argc, argv);
}
