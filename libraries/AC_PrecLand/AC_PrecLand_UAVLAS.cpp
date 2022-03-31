#include <AP_HAL/AP_HAL.h>
#include "AC_PrecLand_UAVLAS.h"

AC_PrecLand_UAVLAS::AC_PrecLand_UAVLAS(const AC_PrecLand& frontend, AC_PrecLand::precland_state& state)
    : AC_PrecLand_Backend(frontend, state),
      uavlas()
{
}

void AC_PrecLand_UAVLAS::init()
{
    uavlas.init(get_bus());
}

void AC_PrecLand_UAVLAS::update()
{
    uavlas.update();
    _state.healthy = uavlas.healthy();
    if (uavlas.last_update_ms() != _los_meas_time_ms) {
        _have_los_meas = uavlas.have_los_meas();
        _los_meas_time_ms = uavlas.last_update_ms() ;
    }
}

bool AC_PrecLand_UAVLAS::get_abs_target(Vector3f& pos_ned,Vector3f& vel_ned)
{
    return uavlas.get_abs_target(pos_ned,vel_ned);
}
bool AC_PrecLand_UAVLAS::get_rel_target(Vector3f& pos_ned,Vector3f& vel_ned)
{
    return uavlas.get_rel_target(pos_ned,vel_ned);
}
bool AC_PrecLand_UAVLAS::get_los_body(Vector3f& ret)
{
    return false;
}