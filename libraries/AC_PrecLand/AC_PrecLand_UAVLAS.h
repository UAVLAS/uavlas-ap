#pragma once

#include <AP_Math/AP_Math.h>
#include <AC_PrecLand/AC_PrecLand_Backend.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
 #include <AP_UAVLAS/AP_UAVLAS_SITL.h>
#else
 #include <AP_UAVLAS/AP_UAVLAS.h>
#endif

/*
 * AC_PrecLand_UAVLAS - implements precision landing using target vectors provided
 *                         by a UAVLAS sensor over I2C
 */

class AC_PrecLand_UAVLAS : public AC_PrecLand_Backend
{
public:
    AC_PrecLand_UAVLAS(const AC_PrecLand& frontend, AC_PrecLand::precland_state& state);
    
    void init() override;
    
    void update() override;

    // provides a relative position in NED of landing target
    // returns relative position
    bool get_pos_rel_ned_target(Vector3f& pos_ned) override;

    // provides a unit vector towards the target in body frame
    //  returns same as have_los_meas()
    bool get_los_body(Vector3f& ret) override {return false;};

    // returns system time in milliseconds of last los measurement
    uint32_t los_meas_time_ms() override {return _los_meas_time_ms;};
    
    // return true if there is a valid los measurement available
    bool have_los_meas() override {return _have_los_meas;};

private:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    //AP_UAVLAS_SITL UAVLAS;
#else
    AP_UAVLAS_I2C uavlas;
#endif
    bool                _have_los_meas;
    bool                _have_los_body_meas;
    bool                _have_los_ned_meas;
    bool                _have_los_frd_meas;             
    uint32_t            _los_meas_time_ms;   
};
