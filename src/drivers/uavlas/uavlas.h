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
 * @file uavlas.h
 * @author Yury Kapacheuski
 *
 * Driver for an ULS-QR1 UAVLAS sensor connected via I2C.
 *
 * Created on: Febryary 21, 2021
 **/

#include <string.h>

#include <drivers/device/i2c.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/i2c_spi_buses.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>

#include <uORB/topics/landing_target_pose.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/uavlas_report.h>

#include <px4_platform_common/module_params.h>

#define UAVLAS_I2C_ADDRESS	  0x56
#define UAVLAS_I2C_HMI_ADDR	  0x80 // Who am i register

#define UAVLAS_I2C_RAW_DATA  	  0x00 // Read  RAW data form sensor
#define UAVLAS_I2C_VEHICLE_INFO   0x10 // Write vehicle status to sensor for processing;

#define UAVLAS_I2C_ULSQR1R1_HMI	  0x01 // Who am i register

#define UAVLAS_UPDATE_INTERVAL_US 50000

#define ULS_STATUS_QR1R1_CARRIER_OK (1<<0)
#define ULS_STATUS_QR1R1_SQ_OK      (1<<1)
#define ULS_STATUS_QR1R1_POS_OK     (1<<2)
#define ULS_STATUS_QR1R1_VEL_OK     (1<<3)
#define ULS_STATUS_QR1R1_GU_IMU_OK  (1<<4)
#define ULS_STATUS_QR1R1_MRX_OK     (1<<5)
#define ULS_STATUS_QR1R1_MRXYAW_OK  (1<<6)
#define ULS_STATUS_QR1R1_REL_NED_OK (1<<7)
#define ULS_STATUS_QR1R1_REL_FRD_OK (1<<8)
#define ULS_STATUS_QR1R1_LLM_OK     (1<<9)
#define ULS_STATUS_QR1R1_ABS_NED_OK             (1<<10)
#define ULS_STATUS_VEHICLE_ABS_NED_OK           (1<<11)
#define ULS_STATUS_QR1R1_ABS_NED_BEACON         (1<<12)
#define ULS_STATUS_QR1R1_ABS_NED_PREDICTION     (1<<13)
#define ULS_STATUS_QR1R1_ABS_NED_PLATFORM       (1<<14)


#define ULS_VEHICLE_NE_VALID       (1<<0)
#define ULS_VEHICLE_VNE_VALID      (1<<1)
#define ULS_VEHICLE_HEADING_VALID  (1<<2)
#define ULS_VEHICLE_Z_VALID        (1<<3)
#define ULS_VEHICLE_VZ_VALID       (1<<4)
#define ULS_VEHICLE_PLATFORM_POS_VALID (1<<5)
#define ULS_VEHICLE_PLATFORM_VEL_VALID (1<<6)

#define ULS_VEHICAL_POS_VALID_MASK (ULS_VEHICLE_NE_VALID|ULS_VEHICLE_VNE_VALID|ULS_VEHICLE_HEADING_VALID)



#define ULS_SETTING_DROP_ARM  (5)

namespace uavlas
{

typedef struct __attribute__((packed)){
    uint16_t  guid;   // Ground unit ID
    uint16_t  status; // Status of receiver
    // Raw sensor Data
    float   pos[3]; // Relative Position of receiver in target frame [m]
    float   vel[3]; // Ralative velocity of  receiver in target frame [m/s]
    float   gimu[3];// Orientation of ground unit in [deg]
    float   mrxyaw; // Orientation of the target [deg]
     // Estimated Position Data
    float  rel_pos_ned[3]; // Relative Position of target in NED (North East Down)[m]
    float  rel_vel_ned[3]; // Relative velocity of target in NED (North East Down)[m/s]
    float  abs_pos_ned[3]; // Absolute Position of target in NED (North East Down)ground fixed[m]
    float  abs_vel_ned[3]; // Absolute velocity of target in NED (North East Down) ground fixed[m/s]
    // Status information
    uint8_t   sq;     // Signal quality  [%]
    uint8_t   ss;     // Signal saturation [%]
    uint8_t   sl;     // Signal level [%]
    uint16_t  crc;    // Contol summ for packet checking
}__cpx_uls_data_packet;

typedef struct __attribute__((packed)){
    uint8_t status;
    float  heading; // Vehicle heading information
     // Estimated Position Data
    float  abs_pos_ned[3]; // Absolute Vehicle Position in NED (North East Down) ground fixed [m]
    float  abs_vel_ned[3]; // Absolute Vehicle velocity in NED (North East Down) ground fixed [m/s]
     // Provide platform data if avialible
    // It will be uset to calculate command ned if data not avialible brom beacon
    float  platform_heading; // Platform heading information
    float  platform_abs_pos_ned[3]; // Absolute Platform position in NED (North East Down) ground fixed [m]
    float  platform_abs_vel_ned[3]; // Absolute Platform velocity in NED (North East Down) ground fixed [m/s]
    uint16_t  crc;    // Contol summ for packet checking
}__cpx_uls_vehicle_info_packet;

class UAVLAS : public device::I2C, public I2CSPIDriver<UAVLAS>, public ModuleParams
{
public:
	UAVLAS(I2CSPIBusOption bus_option, const int bus, int bus_frequency, const int address);
	~UAVLAS() override = default;

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);

	static void print_usage();

	int init() override;
	int probe() override;
	void print_status() override;
	//  void update();
	void RunImpl();
	void parameters_update(int parameter_update_sub, bool force = false);
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::ULS_ENABLED>) _uls_enabled,
		(ParamInt<px4::params::ULS_PFM_MODE>) _uls_pmode,
		(ParamInt<px4::params::ULS_ORIENT_MODE>) _uls_omode,
		(ParamInt<px4::params::ULS_PROVIDE_AGL>) _uls_provide_agl,
		(ParamFloat<px4::params::ULS_DROP_ALT>) _uls_drop_alt
	)
private:
	int 	read_device();
	int 	send_navigation();
	void 	updateNavigation();
	int 	read_device_block(struct uavlas_report_s *block);
	void 	update_topics();

	uORB::Publication<uavlas_report_s> _uavlas_report_topic{ORB_ID(uavlas_report)};
	struct uavlas_report_s orb_report;

	uORB::Publication<distance_sensor_s> _distance_sensor_topic{ORB_ID(distance_sensor)};

	uORB::Subscription _vehicleLocalPositionSub{ORB_ID(vehicle_local_position)};
	vehicle_local_position_s  _vehicleLocalPosition{};

	uORB::Subscription _attitudeSub{ORB_ID(vehicle_attitude)};
	vehicle_attitude_s _vehicleAttitude{};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};


	uORB::Subscription _parameterSub{ORB_ID(parameter_update)};


	uORB::Publication<landing_target_pose_s> _targetPosePub{ORB_ID(landing_target_pose)};
	landing_target_pose_s _target_pose{};
	uORB::Publication<vehicle_land_detected_s> _vehicle_land_detected_pub{ORB_ID(vehicle_land_detected)};
	vehicle_land_detected_s _land_detected = {
		.timestamp = 0,
		.alt_max = -1.0f,
		.freefall = false,
		.ground_contact = true,
		.maybe_landed = true,
		.landed = true,
	};
	bool _isSensorConnected;
	bool _vehicleLocalPosition_valid{false};
	bool _vehicleAttitude_valid{false};
	uint8_t _nav_state{0};
	int _drop_arming;
	int _read_failures;
	int _write_failures;
	int _crc_failures;

	int _parameter_update_sub;

};
}

