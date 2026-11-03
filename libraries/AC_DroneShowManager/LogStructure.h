#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_DRONE_SHOW \
    LOG_DRONE_SHOW_MSG, \
    LOG_FENCE_STATUS_MSG, \
    LOG_DRONE_SHOW_EVENT_MSG, \
    LOG_SCREENPLAY_ENTRY_MSG, \
    LOG_CRTH_TRIGGER_MSG, \
    LOG_SHOW_REFERENCE_MSG, \
    LOG_SHOW_IDENTITY_MSG, \
    LOG_SHOW_CLOCK_MSG, \
    LOG_SHOW_CLOCK_COMMIT_MSG, \
    LOG_SHOW_SCHEDULE_MSG

// @LoggerMessage: SHOW
// @Description: Drone show mode information
// @Field: TimeUS: Time since system startup
// @Field: ClockMS: Time since the start of the show in wall clock time
// @Field: Stage: Current stage of the show
// @Field: Scene: Scene index in the current show
// @Field; ShowClockMS: Time on the show clock in the current scene
// @Field: R: Red component of current color in show
// @Field: G: Green component of current color in show
// @Field: B: Blue component of current color in show
// @Field: HDist: Horizontal distance from desired position
// @Field: VDist: Vertical distance from desired position

// drone show mode logging
struct PACKED log_DroneShowStatus {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int32_t wall_clock_ms;
    uint8_t stage;
    uint8_t scene;
    int32_t show_clock_ms;
    float x;
    float y;
    float z;
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    float h_dist;
    float v_dist;
};

// @LoggerMessage: FNCS
// @Description: Geofence status information
// @Field: TimeUS: Time since system startup
// @Field: GeoEn: Bitmask of enabled geofences
// @Field: GeoB: Bitmask of current geofence breaches
// @Field: GeoCnt: Number of geofence breaches
// @Field: HardB: Status of hard geofence (0: OK, 1: breached, 2: action taken)
// @Field: BubbleB: Status of bubble geofence (0: OK, 1: breached, 2: action taken)
// @Field: BubbleCnt: Number of bubble geofence actions taken

// fence status logging
struct PACKED log_FenceStatus {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t geo_enabled;
    uint8_t geo_breaches;
    uint16_t geo_breach_count;
    uint8_t hard_breach_state;
    uint8_t bubble_breach_state;
    uint16_t bubble_breach_count;
};

// @LoggerMessage: SBEV
// @Description: Skybrush show file event execution log
// @Field: TimeUS: Time since system startup
// @Field: ClockMS: Time since the start of the show in wall clock time
// @Field: Scene: Scene index in the current show
// @Field; ShowClockMS: Time on the show clock in the current scene
// @Field: Type: Type of the event from the show file
// @Field: Subtype: Subtype of the event from the show file
// @Field: Payload: Payload of the event from the show file as uint32_t
// @Field: Result: Result of the event execution

// drone show events
struct PACKED log_DroneShowEvent {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int32_t wall_clock_ms;
    uint8_t scene;
    int32_t show_clock_ms;
    uint8_t type;
    uint8_t subtype;
    uint32_t payload;
    uint8_t result;
};

// @LoggerMessage: SBSP
// @Description: Skybrush screenplay log
// @Field: TimeUS: Time since system startup
// @Field: Seq: Sequence number of the time axis configuration
// @Field: Scene: Index of the scene that this entry refers to
// @Field: Index: Index of the time axis entry in this scene
// @Field: Origin: Number of milliseconds elapsed since the UNIX epoch at the time when the show clock is at 00:00,
// @Field: Duration: Duration of the time axis entry in milliseconds
// @Field: IR: Initial rate of the time axis entry (1 = real time, 0.5 = half speed, 0 = standstill, etc.)
// @Field: FR: Final rate of the time axis entry (1 = real time, 0.5 = half speed, 0 = standstill, etc.)
struct PACKED log_ScreenplayEntry {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t seq_no;
    uint8_t scene;
    uint8_t index;
    uint64_t origin_ms;
    uint32_t duration_ms;
    float initial_rate;
    float final_rate;
};

// @LoggerMessage: CRTH
// @Description: Collective RTH trigger log
// @Field: TimeUS: Time since system startup
// @Field: ClockMS: Time since the start of the show in wall clock time
// @Field: StartMS: Start time of the chosen collective RTH plan
// @Field: StartX: X coordinate of the start of the RTH trajectory
// @Field: StartY: Y coordinate of the start of the RTH trajectory
// @Field: StartZ: Z coordinate of the start of the RTH trajectory
struct PACKED log_CollectiveRTHTrigger {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int32_t wall_clock_ms;
    uint32_t rth_start_time_ms;
    float start_x;
    float start_y;
    float start_z;
};

// @LoggerMessage: SHRF
// @Description: Show trajectory reference and estimated position during the performance, written with every guided mode command when SHOW_OPTIONS bit 3 is set
// @Field: TimeUS: Time since system startup
// @Field: ShowUS: Time on the show trajectory (warped show clock)
// @Field: Src: Source of the reference (0: .skyb, 1: .gdmb)
// @Field: FF: Acceleration feed-forward: bit 0 enabled, bit 1 clipped to SHOW_ACC_FF_MAX in this command (the A* fields are the unclipped reference)
// @Field: PN: Reference position, North of the EKF origin
// @Field: PE: Reference position, East of the EKF origin
// @Field: PU: Reference position, up from the EKF origin
// @Field: VN: Reference velocity, North
// @Field: VE: Reference velocity, East
// @Field: VU: Reference velocity, up
// @Field: AN: Reference acceleration, North
// @Field: AE: Reference acceleration, East
// @Field: AU: Reference acceleration, up
// @Field: N: Estimated position, North of the EKF origin
// @Field: E: Estimated position, East of the EKF origin
// @Field: U: Estimated position, up from the EKF origin
struct PACKED log_ShowReference {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int64_t show_time_us;
    uint8_t source;
    uint8_t accel_ff;
    float ref_n;
    float ref_e;
    float ref_u;
    float vel_n;
    float vel_e;
    float vel_u;
    float acc_n;
    float acc_e;
    float acc_u;
    float n;
    float e;
    float u;
};

// @LoggerMessage: SHID
// @Description: Identity of the loaded show file, written whenever the motors are armed
// @Field: TimeUS: Time since system startup
// @Field: Fmt: Format of the show file in /axio/show.bin (0: none, 1: .skyb, 2: .gdmb)
// @Field: Size: Size of the show file in bytes
// @Field: HCRC: CRC32 stored in the file header
// @Field: FCRC: AP-CRC32 of the whole file, as MAVFTP reports it after an upload
// @Field: Hash: .gdmb show hash (SHA-256, lowercase hex); empty for .skyb
struct PACKED log_ShowIdentity {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t format;
    uint32_t size;
    uint32_t header_crc32;
    uint32_t file_crc32;
    char hash[64];
};

// @LoggerMessage: SCLK
// @Description: Show clock sample received from the positioning tag
// @Field: TimeUS: Time since system startup
// @Field: Show: Show clock time in the sample (UWB cluster clock)
// @Field: Off: Internal clock minus show clock after the sample
// @Field: N: Number of samples in the filter window
// @Field: Acc: Whether the sample was accepted (0: ignored as an outlier)
struct PACKED log_ShowClock {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint64_t show_time_us;
    int64_t offset_us;
    uint8_t samples;
    uint8_t accepted;
};

// @LoggerMessage: SCOM
// @Description: Show start committed on the internal clock
// @Field: TimeUS: Time since system startup
// @Field: Start: Start time on the show clock (UWB cluster clock)
// @Field: LStart: Start time on the internal clock
// @Field: Off: Internal clock minus show clock used for the commit
// @Field: Valid: Whether the show clock was valid at the commit
// @Field: Id: ID of the show schedule that set the start
struct PACKED log_ShowClockCommit {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint64_t start_show_us;
    uint64_t start_local_us;
    int64_t offset_us;
    uint8_t clock_valid;
    uint16_t schedule_id;
};

// @LoggerMessage: SSCH
// @Description: Show schedule received from the ground and what was done with it; repetitions are not logged
// @Field: TimeUS: Time since system startup
// @Field: Id: Schedule ID
// @Field: Start: Start time on the show clock (UWB cluster clock); 0 clears or aborts
// @Field: Act: 0 ignored (start committed), 1 scheduled, 2 cleared, 3 refused (less than 10 s ahead), 4 aborted, 5 no show clock (waits for a repetition), 6 dropped on a show clock timebase step
struct PACKED log_ShowSchedule {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t schedule_id;
    uint64_t start_show_us;
    uint8_t action;
};

#define LOG_STRUCTURE_FROM_DRONE_SHOW \
    { LOG_DRONE_SHOW_MSG, sizeof(log_DroneShowStatus),                  \
      "SHOW", "QiBBifffBBBff", "TimeUS,ClockMS,Stage,Scene,SceneMS,X,Y,Z,R,G,B,HDist,VDist", "ss--smmm---mm", "FC--C--------" }, \
    { LOG_FENCE_STATUS_MSG, sizeof(log_FenceStatus),                    \
      "FNCS", "QBBHBBH", "TimeUS,GeoEn,GeoB,GeoCnt,HardB,BubbleB,BubbleCnt", "s------", "F------" }, \
    { LOG_DRONE_SHOW_EVENT_MSG, sizeof(log_DroneShowEvent),              \
      "SBEV", "QiBiBBIB", "TimeUS,ClockMS,Scene,SceneMS,Type,Subtype,Payload,Result", "ss-s----", "FC-C----" }, \
    { LOG_SCREENPLAY_ENTRY_MSG, sizeof(log_ScreenplayEntry),             \
      "SBSP", "QBBBQIff", "TimeUS,Seq,Scene,Index,Origin,Duration,IR,FR", "s---ss--", "F---CC--" }, \
    { LOG_CRTH_TRIGGER_MSG, sizeof(log_CollectiveRTHTrigger),            \
      "CRTH", "QiIfff", "TimeUS,ClockMS,StartMS,StartX,StartY,StartZ", "sssmmm", "FCC---" }, \
    { LOG_SHOW_REFERENCE_MSG, sizeof(log_ShowReference),                 \
      "SHRF", "QqBBffffffffffff", "TimeUS,ShowUS,Src,FF,PN,PE,PU,VN,VE,VU,AN,AE,AU,N,E,U", "ss--mmmnnnooommm", "FF--000000000000" }, \
    { LOG_SHOW_IDENTITY_MSG, sizeof(log_ShowIdentity),                   \
      "SHID", "QBIIIZ", "TimeUS,Fmt,Size,HCRC,FCRC,Hash", "s-b---", "F-0---" }, \
    { LOG_SHOW_CLOCK_MSG, sizeof(log_ShowClock),                         \
      "SCLK", "QQqBB", "TimeUS,Show,Off,N,Acc", "sss--", "FFF--" }, \
    { LOG_SHOW_CLOCK_COMMIT_MSG, sizeof(log_ShowClockCommit),            \
      "SCOM", "QQQqBH", "TimeUS,Start,LStart,Off,Valid,Id", "ssss--", "FFFF--" }, \
    { LOG_SHOW_SCHEDULE_MSG, sizeof(log_ShowSchedule),                   \
      "SSCH", "QHQB", "TimeUS,Id,Start,Act", "s-s-", "F-F-" }
