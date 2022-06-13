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
 * Uavlas.h - UAVLAS landing system base class fo Ardupilot
 * Created on: Jul 8, 2021
 * Author: Yury Kapacheuski
 */

#pragma once

#include <AP_Math/AP_Math.h>

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

#define ULS_AC_DEBUG 1

class Uavlas
{
public:
    virtual void init(int8_t bus) = 0;
    bool healthy() const { return _flags.healthy; }
    uint32_t last_update_ms() const { return _last_update_ms; }
    virtual bool update() = 0;
    
    // provides a absolute position and velocity in NED of landing target
    //  returns position and velocity
    bool get_abs_target(Vector3f& abs_pos_ned,Vector3f& abs_vel_ned);
    
    // provides a relative position and velocity in NED of landing target
    //  returns relative position and velocity
    bool get_rel_ned_target(Vector3f& rel_pos_ned,Vector3f& rel_vel_ned);

    // provides a relative position in NED of landing target
    //  returns relative position
    bool get_pos_rel_ned_target(Vector3f& rel_pos_ned);


    bool have_los_meas(); 

protected:
    struct AP_UAVLAS_Flags {
        uint8_t healthy : 1; 
    }_flags;

    uint32_t _last_update_ms;

    typedef struct {
        uint32_t timestamp;
        uint16_t status;
        Vector3f abs_pos_ned;
        Vector3f abs_vel_ned;
        Vector3f rel_pos_ned;
        Vector3f rel_vel_ned;
    }uls_plnd_target_data;

    uls_plnd_target_data _target;

};

