/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file uavlas.cpp
 * @author Yury Kapacheuski
 *
 * Driver for an ULS-QR1 UAVLAS sensor connected via I2C.
 *
 * Created on: Febryary 21, 2021
 **/
#include "uavlas.h"
namespace uavlas
{

UAVLAS::UAVLAS(I2CSPIBusOption bus_option, const int bus, int bus_frequency, const int address) :
	I2C(DRV_SENS_DEVTYPE_UAVLAS, MODULE_NAME, bus, address, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus, address),
	ModuleParams(nullptr),
	_isSensorConnected(false),
	_vehicleLocalPosition_valid(false),
	_read_failures(0),
	_write_failures(0),
	_crc_failures(0)
{
}
int UAVLAS::init()
{
	int rez = I2C::init();

	if (rez == OK) {
		_parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
		parameters_update(_parameter_update_sub, true);
		// TODO: add on stop - orb_unsubscribe(parameter_update_sub);
		ScheduleNow();
	}

	return rez;
}
int UAVLAS::probe()
{
	uint8_t hmi;
	uint8_t reg = UAVLAS_I2C_HMI_ADDR;
	_retries = 1;

	if (transfer(&reg, 1, &hmi, 1) != OK) {
		//PX4_INFO("ULS-QR1-R1 connection failure. Check cables and power.");
		return -EIO;
	}

	switch (hmi) {
	case UAVLAS_I2C_ULSQR1R1_HMI:
		PX4_INFO("ULS-QR1-R1 connected.");
		break;

	default:
		PX4_INFO("Unsupported UAVLAS device connected.");
		return -EIO;
	}

	_isSensorConnected = true;
	return PX4_OK;
}
void UAVLAS::print_status()
{
	PX4_INFO("id:%u status:%u sq:%.2f cs:%f sl:%f RE:%d WE:%d CRCE:%d",
		 orb_report.id,
		 orb_report.status,
		 (double)orb_report.sq,
		 (double)orb_report.ss,
		 (double)orb_report.sl,
		 _read_failures,
		 _write_failures,
		 _crc_failures);
}
void UAVLAS::RunImpl()
{
	if (!_isSensorConnected) { return; }

	parameters_update(_parameter_update_sub);

	if (_uls_enabled.get() != 0) {


		if (read_device() == OK) { updateNavigation(); }

	}

	ScheduleDelayed(UAVLAS_UPDATE_INTERVAL_US);
}
void UAVLAS::updateNavigation()
{
	update_topics();
	orb_report.timestamp = hrt_absolute_time();
	_uavlas_report_topic.publish(orb_report);

	if (!_vehicleLocalPosition_valid) { // No position to fix
		return ;
	}

	bool targetAbsPosValid = (orb_report.status & ULS_STATUS_QR1R1_ABS_NED_OK);

	if (targetAbsPosValid && _vehicleLocalPosition.xy_valid && _vehicleLocalPosition.z_valid) {
		// Apply information to landing position
		matrix::Vector3f rel_pos = {orb_report.rel_pos_ned_n, orb_report.rel_pos_ned_e, orb_report.rel_pos_ned_d};
		matrix::Vector3f rel_vel = {orb_report.rel_vel_ned_n, orb_report.rel_vel_ned_e, orb_report.rel_vel_ned_d};

		_target_pose.timestamp = orb_report.timestamp;
		_target_pose.is_static = _uls_pmode.get();

		_target_pose.x_rel = rel_pos(0);
		_target_pose.y_rel = rel_pos(1);
		_target_pose.z_rel = rel_pos(2);

		_target_pose.x_abs = rel_pos(0) + _vehicleLocalPosition.x;
		_target_pose.y_abs = rel_pos(1) + _vehicleLocalPosition.y;
		_target_pose.z_abs = rel_pos(2) + _vehicleLocalPosition.z;;

		_target_pose.cov_x_rel = orb_report.rel_pos_ned_d / 20.0f; // Cov approximation
		_target_pose.cov_y_rel = orb_report.rel_pos_ned_d / 20.0f; // Cov approximation

		_target_pose.rel_pos_valid = true;
		_target_pose.abs_pos_valid = true;


		if (_vehicleLocalPosition.v_xy_valid && _vehicleLocalPosition.v_z_valid) {
			_target_pose.vx_abs = rel_vel(0) + _vehicleLocalPosition.vx;
			_target_pose.vy_abs = rel_vel(1) + _vehicleLocalPosition.vy;
			_target_pose.vx_rel = rel_vel(0);
			_target_pose.vy_rel = rel_vel(1);

			_target_pose.cov_vx_rel = orb_report.rel_pos_ned_d / 20.0f; // Cov approximation
			_target_pose.cov_vy_rel = orb_report.rel_pos_ned_d / 20.0f; // Cov approximation

			_target_pose.rel_vel_valid = true;
			_target_pose.abs_vel_valid = true;
		}

		_targetPosePub.publish(_target_pose);
	}


	if (_uls_provide_agl.get() != 0) {//Provide AGL - set AGL of the system if configured.
		distance_sensor_s distance_report{};
		distance_report.timestamp = orb_report.timestamp;
		distance_report.min_distance = 0.1;
		distance_report.max_distance = 20.0;
		distance_report.current_distance = orb_report.pos_z;
		distance_report.variance = orb_report.pos_z / 10.f;
		distance_report.signal_quality = 100;
		distance_report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_INFRARED;
		/* TODO: the ID needs to be properly set */
		distance_report.id = 0;
		distance_report.orientation = ROTATION_PITCH_270;
		_distance_sensor_topic.publish(distance_report);
	}


	if ((targetAbsPosValid) && (_target_pose.z_rel > 0) &&
	    ((2.f * _uls_drop_alt.get()) > _target_pose.z_rel) &&
	    (_nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND)) {

		if (_drop_arming < ULS_SETTING_DROP_ARM) {
			_drop_arming++;
		}else{

		}

	} else {
		_drop_arming = 0;
	}

	if ((_uls_drop_alt.get() > _target_pose.z_rel) &&
	    (_drop_arming == ULS_SETTING_DROP_ARM) &&
	    (_nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND)) {
		// Drop quad
		_land_detected.landed = true;
		_land_detected.freefall = false;
		_land_detected.maybe_landed = true;
		_land_detected.ground_contact = true;
		_land_detected.alt_max = _target_pose.z_rel;
		_land_detected.in_ground_effect = false;
		_land_detected.timestamp = hrt_absolute_time();
		PX4_INFO("Drop Executed.");
		_vehicle_land_detected_pub.publish(_land_detected);
	}


}
int UAVLAS::read_device()
{
	orb_report.timestamp = hrt_absolute_time();

	if (send_navigation() != OK) {
		return -ENOTTY;
	}

	if (read_device_block(&orb_report) != OK) {
		return -ENOTTY;
	}

	_uavlas_report_topic.publish(orb_report);
	return OK;
}
int UAVLAS::send_navigation()
{


	uint8_t buf[sizeof(__cpx_uls_vehicle_info_packet) + 1];
	__cpx_uls_vehicle_info_packet *vehicle = (__cpx_uls_vehicle_info_packet *)&buf[1];


	buf[0] = UAVLAS_I2C_VEHICLE_INFO;

	vehicle->status  = (_vehicleLocalPosition.xy_valid )? (ULS_VEHICLE_NE_VALID) : 0;
	vehicle->status |= (_vehicleLocalPosition.z_valid)? ULS_VEHICLE_Z_VALID:0;
	vehicle->status |= (_vehicleLocalPosition.v_xy_valid) ? (ULS_VEHICLE_VNE_VALID) : 0;
	vehicle->status |= (_vehicleLocalPosition.z_valid) ? (ULS_VEHICLE_VZ_VALID) : 0;
	vehicle->status |= ULS_VEHICLE_HEADING_VALID;

	vehicle->heading = _vehicleLocalPosition.heading;

	vehicle->abs_pos_ned[0] = _vehicleLocalPosition.x;
	vehicle->abs_pos_ned[1] = _vehicleLocalPosition.y;
	vehicle->abs_pos_ned[2] = _vehicleLocalPosition.z;

	vehicle->abs_vel_ned[0] = _vehicleLocalPosition.vx;
	vehicle->abs_vel_ned[1] = _vehicleLocalPosition.vy;
	vehicle->abs_vel_ned[2] = _vehicleLocalPosition.vz;

	vehicle->crc = 0;
    	uint8_t *pxPack = (uint8_t *)vehicle;
    	for (uint32_t i = 0; i < sizeof(__cpx_uls_vehicle_info_packet) - 2; i++) {
        	vehicle->crc += *pxPack++;
    	}

	int status = transfer(buf, sizeof(__cpx_uls_vehicle_info_packet)+1, nullptr, 0);

	if (status != OK) {
		_write_failures++;
		return status;
	}

	return status;

}
int UAVLAS::read_device_block(struct uavlas_report_s *block)
{
	__cpx_uls_data_packet sensor;
	memset(&sensor, 0, sizeof sensor);
	uint8_t reg = UAVLAS_I2C_RAW_DATA;
	int status = transfer(&reg, 1, (uint8_t *)&sensor, sizeof sensor);

	if (status != OK) {
		_read_failures++;
		return status;
	}

	uint16_t tcrc = 0;
	uint8_t *pxPack = (uint8_t *)&sensor;

	for (uint i = 0; i < sizeof(sensor) - 2; i++) {
		tcrc += *pxPack++;
	}

	if (tcrc != sensor.crc) {
		_crc_failures++;
		return -EIO;
	}

	block->id = sensor.guid;
	block->status = sensor.status;

	block->pos_x = sensor.pos[0] ;
	block->pos_y = sensor.pos[1] ;
	block->pos_z = sensor.pos[2] ;
	block->vel_x = sensor.vel[0] ;
	block->vel_y = sensor.vel[1] ;
	block->vel_z = sensor.vel[2] ;
	block->gimu_roll = sensor.gimu[0] ;
	block->gimu_pitch = sensor.gimu[1];
	block->gimu_yaw = sensor.gimu[2] ;
	block->mrxyaw = sensor.mrxyaw ;

	block->sq   = (float)sensor.sq;
	block->ss   = (float)sensor.ss;
	block->sl   = (float)sensor.sl;

	block->rel_pos_ned_n = sensor.rel_pos_ned[0];
	block->rel_pos_ned_e = sensor.rel_pos_ned[1];
	block->rel_pos_ned_d = sensor.rel_pos_ned[2];

	block->rel_vel_ned_n = sensor.rel_vel_ned[0];
	block->rel_vel_ned_e = sensor.rel_vel_ned[1];
	block->rel_vel_ned_d = sensor.rel_vel_ned[2];

	block->abs_pos_ned_n = sensor.abs_pos_ned[0];
	block->abs_pos_ned_e = sensor.abs_pos_ned[1];
	block->abs_pos_ned_d = sensor.abs_pos_ned[2];

	block->abs_vel_ned_n = sensor.abs_vel_ned[0];
	block->abs_vel_ned_e = sensor.abs_vel_ned[1];
	block->abs_vel_ned_d = sensor.abs_vel_ned[2];

	block->receiver_id_valid = (block->status & ULS_STATUS_QR1R1_CARRIER_OK);
	block->abs_ned_valid = (block->status & ULS_STATUS_QR1R1_ABS_NED_OK);
	block->abs_ned_prediction_active = (block->status & ULS_STATUS_QR1R1_ABS_NED_PREDICTION);


	return status;
}
I2CSPIDriverBase *UAVLAS::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
				      int runtime_instance)
{
	UAVLAS *instance = new UAVLAS(iterator.configuredBusOption(), iterator.bus(), cli.bus_frequency, cli.i2c_address);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (instance->init() != PX4_OK) {
		delete instance;
		return nullptr;
	}

	return instance;
}
void UAVLAS::print_usage()
{
	PRINT_MODULE_USAGE_NAME("uavlas", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(UAVLAS_I2C_ADDRESS);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

void UAVLAS::parameters_update(int parameter_update_sub, bool force)
{
	bool updated;
	struct parameter_update_s param_upd;

	// Check if any parameter updated
	orb_check(parameter_update_sub, &updated);

	// If any parameter updated copy it to: param_upd
	if (updated) {
		orb_copy(ORB_ID(parameter_update), parameter_update_sub, &param_upd);
	}

	if (force || updated) {
		// If any parameter updated, call updateParams() to check if
		// this class attributes need updating (and do so).
		updateParams();
	}
}

void UAVLAS::update_topics()
{
	_vehicleLocalPosition_valid = _vehicleLocalPositionSub.update(&_vehicleLocalPosition);
	_vehicleAttitude_valid = _attitudeSub.update(&_vehicleAttitude);

	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status{};
		_vehicle_status_sub.copy(&vehicle_status);
		_nav_state = vehicle_status.nav_state;
	}


}

}
