#include "AC_DroneShowManager.h"
#include "DroneShow_Constants.h"

PostAction AC_DroneShowManager::get_action_at_end_of_show() const
{
    switch (_params.post_action) {
        case PostAction_Land:
            return PostAction_Land;

        case PostAction_Loiter:
            return PostAction_Loiter;

        case PostAction_RTL:
            return PostAction_RTL;

        case PostAction_RTLOrLand:
            return (
                _is_at_takeoff_position_xy(2 * DEFAULT_START_END_XY_DISTANCE_THRESHOLD_METERS) &&
                _trajectory_is_circular
            ) ? PostAction_RTL : PostAction_Land;

        default:
            // Legacy behaviour when we did not have a parameter for the
            // post-show action
            return PostAction_Land;
    }
}

bool AC_DroneShowManager::get_landing_position_NEU_cm(Vector3f& pos)
{
    sb_trajectory_t* trajectory;
    sb_trajectory_player_t player;
    sb_vector3_with_yaw_t end_with_yaw;
    sb_vector3_t end;
    Location loc;
    bool success;

    if (!loaded_show_data_successfully() || !is_trajectory_plausible()) {
        return false;
    }

    // Evaluate the raw trajectory at its end with a dedicated player instead
    // of going through the show controller: the show controller's input is
    // screenplay wall-clock time, which may be warped by a non-identity time
    // axis (suspension / resume), and the screenplay scene duration goes
    // stale when the trajectory end is rewritten for circular trajectories
    // at takeoff. The raw trajectory is the single source of truth for where
    // the show intends the drone to land.
    trajectory = sb_screenplay_scene_get_trajectory(&_main_show_scene);
    if (trajectory == nullptr) {
        return false;
    }

    if (sb_trajectory_player_init(&player, trajectory) != SB_SUCCESS) {
        return false;
    }

    // The player clamps to the last trajectory point when evaluated at or
    // beyond the end of the trajectory
    success = sb_trajectory_player_get_position_at(
        &player, _trajectory_stats.duration_sec, &end_with_yaw
    ) == SB_SUCCESS;
    sb_trajectory_player_destroy(&player);

    if (!success) {
        return false;
    }

    end.x = end_with_yaw.x;
    end.y = end_with_yaw.y;
    end.z = end_with_yaw.z;
    _show_coordinate_system.convert_show_to_global_coordinate(end, loc);
    return loc.get_vector_from_origin_NEU(pos);
}

float AC_DroneShowManager::get_landing_speed_m_sec() const {
    float value = 0.0f;

    if (AP_Param::get("LAND_SPEED", value)) {
        if (value >= 0.0f && isfinite(value)) {
            return value / 100.0f; // Convert from cm/s to m/s
        }
    }

    return DEFAULT_LANDING_SPEED_METERS_PER_SEC;
}
