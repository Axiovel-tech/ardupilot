#pragma once

#include <AP_Logger/LogStructure.h>
#include "AP_VisualOdom_config.h"

#define LOG_IDS_FROM_VISUALODOM \
    LOG_VISUALODOM_MSG, \
    LOG_VISUALPOS_MSG, \
    LOG_VISUALVEL_MSG, \
    LOG_VISUALCOV_MSG

// @LoggerMessage: VISO
// @Description: Visual Odometry
// @Field: TimeUS: System time
// @Field: dt: Time period this data covers
// @Field: AngDX: Angular change for body-frame roll axis
// @Field: AngDY: Angular change for body-frame pitch axis
// @Field: AngDZ: Angular change for body-frame z axis
// @Field: PosDX: Position change for body-frame X axis (Forward-Back)
// @Field: PosDY: Position change for body-frame Y axis (Right-Left)
// @Field: PosDZ: Position change for body-frame Z axis (Down-Up)
// @Field: conf: Confidence
struct PACKED log_VisualOdom {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float time_delta;
    float angle_delta_x;
    float angle_delta_y;
    float angle_delta_z;
    float position_delta_x;
    float position_delta_y;
    float position_delta_z;
    float confidence;
};

// @LoggerMessage: VISP
// @Description: Vision Position
// @Field: TimeUS: System time
// @Field: RTimeUS: Remote system time
// @Field: CTimeMS: Corrected system time
// @Field: PX: Position X-axis (North-South)
// @Field: PY: Position Y-axis (East-West)
// @Field: PZ: Position Z-axis (Down-Up)
// @Field: R: Roll lean angle
// @Field: P: Pitch lean angle
// @Field: Y: Yaw angle
// @Field: PErr: Legacy scalar position estimate error
// @Field: PErrZ: Effective Down position error supplied to EKF3
// @Field: AErr: Attitude estimate error
// @Field: Rst: Position reset counter
// @Field: Ign: Ignored
// @Field: Q: Quality
struct PACKED log_VisualPosition {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint64_t remote_time_us;
    uint32_t time_ms;
    float pos_x;
    float pos_y;
    float pos_z;
    float roll;     // degrees
    float pitch;    // degrees
    float yaw;      // degrees
    float pos_err;   // legacy scalar, meters
    float pos_err_z; // Down, meters
    float ang_err;   // radians
    uint8_t reset_counter;
    uint8_t ignored;
    int8_t quality;
};

// @LoggerMessage: VISC
// @Description: Vision Position Covariance
// @Field: TimeUS: System time
// @Field: RTimeUS: Remote system time
// @Field: PEL: Legacy scalar position error supplied to EKF2
// @Field: PEN: Effective North position error supplied to EKF3
// @Field: PEE: Effective East position error supplied to EKF3
// @Field: PED: Effective Down position error supplied to EKF3
// @Field: CXX: Reported X-axis position variance
// @Field: CXY: Reported X/Y position covariance
// @Field: CXZ: Reported X/Z position covariance
// @Field: CYY: Reported Y-axis position variance
// @Field: CYZ: Reported Y/Z position covariance
// @Field: CZZ: Reported Z-axis position variance
// @Field: CVD: Covariance diagonal is valid and was used for PEN, PEE and PED
struct PACKED log_VisualPositionCovariance {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint64_t remote_time_us;
    float pos_err_legacy; // meters
    float pos_err_n;      // North, meters
    float pos_err_e;      // East, meters
    float pos_err_d;      // Down, meters
    float covariance_xx;  // meters squared
    float covariance_xy;  // meters squared
    float covariance_xz;  // meters squared
    float covariance_yy;  // meters squared
    float covariance_yz;  // meters squared
    float covariance_zz;  // meters squared
    uint8_t covariance_used;
};

// @LoggerMessage: VISV
// @Description: Vision Velocity
// @Field: TimeUS: System time
// @Field: RTimeUS: Remote system time
// @Field: CTimeMS: Corrected system time
// @Field: VX: Velocity X-axis (North-South)
// @Field: VY: Velocity Y-axis (East-West)
// @Field: VZ: Velocity Z-axis (Down-Up)
// @Field: VErr: Velocity estimate error
// @Field: Rst: Velocity reset counter
// @Field: Ign: Ignored
// @Field: Q: Quality
struct PACKED log_VisualVelocity {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint64_t remote_time_us;
    uint32_t time_ms;
    float vel_x;
    float vel_y;
    float vel_z;
    float vel_err;
    uint8_t reset_counter;
    uint8_t ignored;
    int8_t quality;
};

#if HAL_VISUALODOM_ENABLED
#define LOG_STRUCTURE_FROM_VISUALODOM \
    { LOG_VISUALODOM_MSG, sizeof(log_VisualOdom), \
      "VISO", "Qffffffff", "TimeUS,dt,AngDX,AngDY,AngDZ,PosDX,PosDY,PosDZ,conf", "ssrrrmmm-", "FF000000-" }, \
    { LOG_VISUALPOS_MSG, sizeof(log_VisualPosition), \
      "VISP", "QQIfffffffffBBb", "TimeUS,RTimeUS,CTimeMS,PX,PY,PZ,R,P,Y,PErr,PErrZ,AErr,Rst,Ign,Q", "sssmmmddhmmd--%", "FFC000000000--0" }, \
    { LOG_VISUALVEL_MSG, sizeof(log_VisualVelocity), \
      "VISV", "QQIffffBBb", "TimeUS,RTimeUS,CTimeMS,VX,VY,VZ,VErr,Rst,Ign,Q", "sssnnnn--%", "FFC0000--0" }, \
    { LOG_VISUALCOV_MSG, sizeof(log_VisualPositionCovariance), \
      "VISC", "QQffffffffffB", "TimeUS,RTimeUS,PEL,PEN,PEE,PED,CXX,CXY,CXZ,CYY,CYZ,CZZ,CVD", "ssmmmm-------", "FF0000000000-" },
#else
#define LOG_STRUCTURE_FROM_VISUALODOM
#endif
