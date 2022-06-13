/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * AP_UAVLAS_I2C.cpp
 * Created on: Jul 8, 2021
 * Author: Yury Kapacheuski
 */

#include <stdio.h>
#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_HAL/I2CDevice.h>

#include "AP_UAVLAS_I2C.h"
#include <GCS_MAVLink/GCS.h>


extern const AP_HAL::HAL& hal;

#define UAVLAS_I2C_ADDRESS	      0x56 // UAVLAS i2c device addr  
#define UAVLAS_I2C_HMI_ADDR	      0x80 // Who am i register

#define UAVLAS_I2C_RAW_DATA  	  0x00 // Read  RAW data form sensor
#define UAVLAS_I2C_VEHICLE_INFO   0x10 // Write vehicle status to sensor for processing;

#define UAVLAS_I2C_ULSQR1R1_HMI	  0x01 // Who am i register value for ULS-QR1-R1


void AP_UAVLAS_I2C::init(int8_t bus)
{
    if (bus < 0) {
        bus = 2; //
    }
    _bus = bus;
    _dev = std::move(hal.i2c_mgr->get_device(bus, UAVLAS_I2C_ADDRESS));

    if (!_dev) {
        return;
    }
    _stage = AP_UAVLAS_I2C::INIT;

    _dev->register_periodic_callback(50000, FUNCTOR_BIND_MEMBER(&AP_UAVLAS_I2C::uls_task, void));

}
void AP_UAVLAS_I2C::uls_task(void)
{
    switch(_stage) {
    case INIT:
        _flags.healthy = 0;
        _stage = on_init();
        break;
    case CONNECTED:
        _flags.healthy = 1;
        _stage = on_connected();
        break;
    default: //wtf
        break;
    }
}
AP_UAVLAS_I2C::uls_stage_type_t AP_UAVLAS_I2C::on_init()
{
    uint8_t hmi;
    if(!_dev->read_registers(UAVLAS_I2C_HMI_ADDR, &hmi, 1)) {
        uls_log("UAVLAS: no device on bus %d (try other bus#)",(int)_bus);
        return AP_UAVLAS_I2C::INIT;
    }
    switch (hmi) {
    case UAVLAS_I2C_ULSQR1R1_HMI:
        uls_log("UAVLAS: ULS-QR1-R1 connected.");
        break;
    default:
        uls_log("UAVLAS: Unsupported device on bus.");
        return AP_UAVLAS_I2C::INIT;
    }

    return AP_UAVLAS_I2C::CONNECTED;
}
AP_UAVLAS_I2C::uls_stage_type_t AP_UAVLAS_I2C::on_connected()
{
    struct cpx_uls_data_packet cpx_uls_frame;
    if (!send_vehicle_info()) {
        return AP_UAVLAS_I2C::INIT;
    }
    if (!read_block(cpx_uls_frame)) {
        return AP_UAVLAS_I2C::INIT;
    }

    {
        WITH_SEMAPHORE(sem);

        _target.timestamp = AP_HAL::millis();
        
        if((_target.status & ULS_STATUS_QR1R1_REL_NED_OK) && 
           !(cpx_uls_frame.status & ULS_STATUS_QR1R1_REL_NED_OK)){
            uls_log("UAVLAS: Target lost.");   
           }
         if(!(_target.status & ULS_STATUS_QR1R1_REL_NED_OK) && 
            (cpx_uls_frame.status & ULS_STATUS_QR1R1_REL_NED_OK)){
            uls_log("UAVLAS: Target found.");   
           }  
        _target.status = cpx_uls_frame.status;
        for(int i=0; i<3; i++) {
            _target.rel_pos_ned[i] = cpx_uls_frame.rel_pos_ned[i];
            _target.rel_vel_ned[i] = cpx_uls_frame.rel_vel_ned[i];
            _target.abs_pos_ned[i] = cpx_uls_frame.abs_pos_ned[i];
            _target.abs_vel_ned[i] = cpx_uls_frame.abs_vel_ned[i];

        }
    }
    return AP_UAVLAS_I2C::CONNECTED;
}

bool AP_UAVLAS_I2C::send_vehicle_info()
{
    struct cpx_uls_vehicle_info_packet cpx_uls_vehicle;
    Vector3f curr_pos;
    Vector3f curr_vel;
    cpx_uls_vehicle.status = 0;

    if (AP::ahrs().get_relative_position_NED_origin(curr_pos)) {
        cpx_uls_vehicle.status |= ULS_VEHICLE_NE_VALID|ULS_VEHICLE_Z_VALID;    
    } else {
        Vector2f curr_posNE;
        if(AP::ahrs().get_relative_position_NE_origin(curr_posNE)) {
            cpx_uls_vehicle.status |= ULS_VEHICLE_NE_VALID;
            curr_pos[0] = curr_posNE[0];
            curr_pos[1] = curr_posNE[1];
            curr_pos[2] = 0;
        }
    }

    if(AP::ahrs().get_velocity_NED(curr_vel)) {
        cpx_uls_vehicle.status |= ULS_VEHICLE_VNE_VALID|ULS_VEHICLE_VZ_VALID;
    }
    

    for(int i=0; i<3; i++) {
        cpx_uls_vehicle.abs_pos_ned[i] = curr_pos[i];
        cpx_uls_vehicle.abs_vel_ned[i] = curr_vel[i];
    }

    cpx_uls_vehicle.heading = AP::ahrs().get_yaw();
    cpx_uls_vehicle.status |= ULS_VEHICLE_HEADING_VALID;

    cpx_uls_vehicle.crc = 0;
    uint8_t *pxPack = (uint8_t *)&cpx_uls_vehicle;
    for (uint32_t i = 0; i < sizeof(cpx_uls_vehicle) - 2; i++) {
        cpx_uls_vehicle.crc += *pxPack++;
    }
    uint8_t buf[sizeof(cpx_uls_vehicle)+1];
    memcpy(&buf[1],(uint8_t*)&cpx_uls_vehicle,sizeof(cpx_uls_vehicle));

    buf[0] = UAVLAS_I2C_VEHICLE_INFO;

    if (!_dev->transfer(buf,sizeof(cpx_uls_vehicle)+1,nullptr,0 )) {
        uls_log( "UAVLAS: I2C Write error.");
        return false;
    }
    return true;
}
bool AP_UAVLAS_I2C::read_block(struct cpx_uls_data_packet &cpx_uls_frame)
{
    if (!_dev->read_registers(UAVLAS_I2C_RAW_DATA,
                              (uint8_t*)&cpx_uls_frame,
                              sizeof(cpx_uls_data_packet))) {
        uls_log( "UAVLAS: I2C Read error.");
        return false;
    }

    _last_read_ms = AP_HAL::millis();

    uint16_t tcrc = 0;
    uint8_t *pxPack = (uint8_t *)&cpx_uls_frame;

    for (uint32_t i = 0; i < sizeof(cpx_uls_data_packet) - 2; i++) {
        tcrc += *pxPack++;
    }
    if (tcrc != cpx_uls_frame.crc) {
        _crc_failures++;
        uls_log("UAVLAS: IO CRC error.");
        return false;
    }

    // UAVLAS Status logging
    struct log_UavlasStatus  pkt_status = {
        LOG_PACKET_HEADER_INIT(LOG_ULSS_MSG),
        time_us     : AP_HAL::micros64(),
        guid     :cpx_uls_frame.guid,   // Ground unit ID
        status   : cpx_uls_frame.status, // Status of receiver
        // Raw sensor Data
        // Relative Position of receiver in target frame [m]
        pos      : {cpx_uls_frame.pos[0],cpx_uls_frame.pos[1],cpx_uls_frame.pos[2]},
        // Ralative velocity of  receiver in target frame [m/s]
        vel      : {cpx_uls_frame.vel[0],cpx_uls_frame.vel[1],cpx_uls_frame.vel[2]},
        // Orientation of ground unit in [deg]
        gimu     : {cpx_uls_frame.gimu[0],cpx_uls_frame.gimu[1],cpx_uls_frame.gimu[2]},
        mrxyaw   : cpx_uls_frame.mrxyaw, // Orientation of the target [deg]
        // Status information
        sq       : cpx_uls_frame.sq,     // Signal quality  [%]
        ss       : cpx_uls_frame.ss,     // Signal saturation [%]
        sl       : cpx_uls_frame.sl     // Signal level [%]
    };

    // UAVLAS Status logging
    struct log_UavlasTarget pkt_target = {
        LOG_PACKET_HEADER_INIT(LOG_ULST_MSG),
        time_us     : AP_HAL::micros64(),
        // Estimated Position Data
        // Absolute Position of target in NED (North East Down)ground fixed[m]
        abs_pos_ned: {cpx_uls_frame.abs_pos_ned[0],cpx_uls_frame.abs_pos_ned[1],cpx_uls_frame.abs_pos_ned[2]},
        // Absolute velocity of target in NED (North East Down) ground fixed[m/s]
        abs_vel_ned: {cpx_uls_frame.abs_vel_ned[0],cpx_uls_frame.abs_vel_ned[1],cpx_uls_frame.abs_vel_ned[2]},
        // Relative Position of target in NED (North East Down)[m]
        rel_pos_ned: {cpx_uls_frame.rel_pos_ned[0],cpx_uls_frame.rel_pos_ned[1],cpx_uls_frame.rel_pos_ned[2]},
        // Relative velocity of target in NED (North East Down)[m/s]
        rel_vel_ned: {cpx_uls_frame.rel_vel_ned[0],cpx_uls_frame.rel_vel_ned[1],cpx_uls_frame.rel_vel_ned[2]},
    };

    AP::logger().WriteBlock(&pkt_target, sizeof(pkt_target));
    AP::logger().WriteBlock(&pkt_status, sizeof(pkt_status));

    return true;
}

bool AP_UAVLAS_I2C::msg_timeout(void)
{
    uint32_t current_time = AP_HAL::millis();
    if (current_time - _msg_last_time > 5000) {
        _msg_last_time = current_time;
        return true;
    }
    return false;
}

bool AP_UAVLAS_I2C::update()
{
    bool new_data = false;
    if (!_dev) {
        return false;
    }

    WITH_SEMAPHORE(sem);

    if (_last_update_ms != _target.timestamp) {
        new_data = true;
    }
    _last_update_ms = _target.timestamp;
    _flags.healthy = (AP_HAL::millis() - _last_read_ms < 100);

    // return true if new data found
    return new_data;
}

void AP_UAVLAS_I2C::uls_log(const char* format, ...)
{
    char msg[65] {};
    va_list arg_list;
    va_start(arg_list, format);
    hal.util->vsnprintf(msg, sizeof(msg), format, arg_list);
    if(msg_timeout()) {
        gcs().send_textv(MAV_SEVERITY_INFO, format, arg_list);
    }
    va_end(arg_list);
    AP::logger().Write_Message(msg);

}