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
 * AP_UAVLAS_I2C.h
 * Created on: Jul 8, 2021
 * Author: Yury Kapacheuski
 */

#pragma once


#include "Uavlas.h"
#include <AP_HAL/AP_HAL.h>

class AP_UAVLAS_I2C : public Uavlas
{
public:
    AP_UAVLAS_I2C() {
        _bus = -1;
    };
    void init(int8_t bus) override;
    bool update() override;
    typedef enum {
        INIT=0,
        CONNECTED,
    } uls_stage_type_t;
private:
    AP_HAL::OwnPtr<AP_HAL::Device>  _dev;

    struct PACKED cpx_uls_data_packet {
        uint16_t  guid;   // Ground unit ID
        uint16_t  status; // Status of receiver
        // Raw sensor Data
        float   pos[3]; // Relative Position of receiver in target frame [m]
        float   vel[3]; // Relative velocity of  receiver in target frame [m/s]
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
    } ;

    struct PACKED cpx_uls_vehicle_info_packet {
        uint8_t status;
        float  heading; // Vehicle heading information
        // Estimated Position Data
        float  abs_pos_ned[3]; // Absolute Vehicle Position in NED (North East Down) ground fixed [m]
        float  abs_vel_ned[3]; // Absolute Vehicle velocity in NED (North East Down) ground fixed [m/s]
        // Provide platform data if avialible
        // It will be used to calculate command ned if data not avialible from beacon
        float  platform_heading; // Platform heading information
        float  platform_abs_pos_ned[3]; // Absolute Platform position in NED (North East Down) ground fixed [m]
        float  platform_abs_vel_ned[3]; // Absolute Platform velocity in NED (North East Down) ground fixed [m/s] 
        uint16_t  crc;    // Contol summa for packet checking
    } ;


    int  _crc_failures;
    int _bus;

    uint32_t _msg_last_time;
    uls_stage_type_t _stage;

    bool timer(void);
    bool read_block(struct cpx_uls_data_packet &cpx_uls_frame);
    bool send_vehicle_info();
    void uls_task(void);
    bool msg_timeout();
    void uls_log(const char* format, ...);
    uls_stage_type_t on_init();
    uls_stage_type_t on_connected();

    HAL_Semaphore sem;
    uint32_t _last_read_ms;
};