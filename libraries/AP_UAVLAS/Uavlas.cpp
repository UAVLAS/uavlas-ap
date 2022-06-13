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
 * Uavlas.cpp
 * Created on: Jul 8, 2021
 * Author: Yury Kapacheuski
 */
#include "Uavlas.h"

bool Uavlas::get_rel_ned_target(Vector3f& rel_pos_ned,Vector3f& rel_vel_ned)
{
    // return false if not healthy or no NED information provided
    if (!_flags.healthy || ((_target.status & ULS_STATUS_QR1R1_REL_NED_OK) == 0)) {
        return false;
    }
    rel_pos_ned = _target.rel_pos_ned;
    rel_vel_ned = _target.rel_vel_ned;
    return true;
}
bool Uavlas::get_pos_rel_ned_target(Vector3f& rel_pos_ned)
{
    // return false if not healthy or no NED information provided
    if (!_flags.healthy || ((_target.status & ULS_STATUS_QR1R1_REL_NED_OK) == 0)) {
        return false;
    }
    rel_pos_ned = _target.rel_pos_ned;
    return true;
}
bool Uavlas::have_los_meas()
{
    return (_target.status & ULS_STATUS_QR1R1_REL_NED_OK);
}