#pragma once

#include <AP_Logger/LogStructure.h>


#define LOG_IDS_FROM_UAVLAS \
    LOG_ULSS_MSG,LOG_ULST_MSG

// @LoggerMessage: ULSS
// @Description: UAVLAS Status data messages
// @URL:
// @Field: TimeUS: Time since system startup
// @Field: guid:  Ground  unit ID
// @Field: Status:  Status of receiver
// @Field: PX: Relative X-Axis Position (in transmitter frame)[m]
// @Field: PY: Relative Y-Axis Position (in transmitter frame)[m]
// @Field: PZ: Relative Z-Axis Position (in transmitter frame)[m]
// @Field: VX: Relative X-Axis Velocity (in target frame)[m/s]
// @Field: VY: Relative Y-Axis Velocity (in target frame)[m/s]
// @Field: VZ: Relative Z-Axis Velocity (in target frame)[m/s]
// @Field: GR: Orientation of ground unit Roll [deg]
// @Field: GP: Orientation of ground unit Pitch [deg]
// @Field: GY: Orientation of ground unit Yaw [deg]
// @Field: MrxY:  Orientation of vehile relative to ground unit Yaw[deg]
// @Field: SQ: Signal quality [%]
// @Field: SS: Signal saturation [%]
// @Field: SL: Signal level [%]

// @LoggerMessage: ULST
// @Description:  UAVLAS NED target position and velocity. 
// @URL:
// @Field: TimeUS: Time since system startup
// @Field: APN: Absolute North Position of target (in ground frame)[m]
// @Field: APE: Absolute East Position of target (in ground frame)[m]
// @Field: APD: Absolute Down Position of target (in ground frame)[m]
// @Field: AVN: Absolute North Velocity of target (in ground frame)[m/s]
// @Field: AVE: Absolute East Velocity of target (in ground frame)[m/s]
// @Field: AVD: Absolute Down Velocity of target (in ground frame)[m/s]      
// @Field: RPN: Relative North Position of target (in vehicle frame)[m]
// @Field: RPE: Relative East Position of target (in vehicle frame)[m]
// @Field: RPD: Relative Down Position of target (in vehicle frame)[m]
// @Field: RVN: Relative North Velocity of target (in vehicle frame)[m/s]
// @Field: RVE: Relative East Velocity of target (in vehicle frame)[m/s]
// @Field: RVD: Relative Down Velocity of target (in vehicle frame)[m/s]


// UAVLAS Status logging
struct PACKED log_UavlasStatus {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t  guid;   // Ground unit ID
    uint16_t  status; // Status of receiver
        // Raw sensor Data
    float   pos[3]; // Relative Position of receiver in target frame [m]
    float   vel[3]; // Ralative velocity of  receiver in target frame [m/s]
    float   gimu[3];// Orientation of ground unit in [deg]
    float   mrxyaw; // Orientation of the target [deg]
        // Status information
    uint8_t   sq;     // Signal quality  [%]
    uint8_t   ss;     // Signal saturation [%]
    uint8_t   sl;     // Signal level [%]
};

// UAVLAS Target logging
struct PACKED log_UavlasTarget {
    LOG_PACKET_HEADER;
    uint64_t time_us;
       // Estimated Position Data
    float  abs_pos_ned[3]; // Absolute Position of target in NED (North East Down)ground fixed[m]
    float  abs_vel_ned[3]; // Absolute velocity of target in NED (North East Down) ground fixed[m/s]
    float  rel_pos_ned[3]; // Relative Position of target in NED (North East Down)[m]
    float  rel_vel_ned[3]; // Relative velocity of target in NED (North East Down)[m/s]
};

#define LOG_STRUCTURE_FROM_UAVLAS  \
        { LOG_ULSS_MSG, sizeof(log_UavlasStatus), \
      "ULSS",  "QHHffffffffffBBB",    "TimeUS,Status,GUID,PX,PY,PZ,VX,VY,VZ,GR,GP,GY,MrxY,SQ,SS,SL", "s--mmmnnndddd---", "F000000000000000", true }, \
      { LOG_ULST_MSG, sizeof(log_UavlasTarget), \
      "ULST",  "Qffffffffffff",    "TimeUS,APN,APE,APD,AVN,AVE,AVD,RPN,RPE,RPD,RVN,RVE,RVD,", "smmmnnnmmmnnn", "F000000000000", true } , 
 

