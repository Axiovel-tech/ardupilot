#include <AP_gtest.h>

#include <AP_Math/AP_Math.h>
#include <AP_Math/vector2.h>
#include <AP_Math/vector3.h>
#include <AP_Math/control.h>

#include <fenv.h>
#include <signal.h>
#include <setjmp.h>

TEST(Control, test_control)
{
    postype_t pos_start = 17;
    float vel_start = 20;
    float accel_start = 1.0;
    const float dt = 0.01;

    // test for update_pos_vel_accel includes update_vel_accel.
    // test unlimited behaviour
    // 1
    float vel = vel_start;
    postype_t pos = pos_start;
    float accel = accel_start;
    update_pos_vel_accel(pos, vel, accel, dt, 0.0, 0.0, 0.0);
    EXPECT_FLOAT_EQ(vel, vel_start + accel * dt);
    EXPECT_FLOAT_EQ(pos, pos_start + vel_start * dt + 0.5 * accel * sq(dt));

    // 2
    vel = vel_start;
    pos = pos_start;
    accel = -accel_start;
    update_pos_vel_accel(pos, vel, accel, dt, 0.0, 0.0, 0.0);
    EXPECT_FLOAT_EQ(vel, vel_start + accel * dt);
    EXPECT_FLOAT_EQ(pos, pos_start + vel_start * dt + 0.5 * accel * sq(dt));

    // error has no impact when not limited
    // 3
    vel = vel_start;
    pos = pos_start;
    accel = accel_start;
    update_pos_vel_accel(pos, vel, accel, dt, 0.0, 1.0, 1.0);
    EXPECT_FLOAT_EQ(vel, vel_start + accel * dt);
    EXPECT_FLOAT_EQ(pos, pos_start + vel_start * dt + 0.5 * accel * sq(dt));

    // 4
    vel = vel_start;
    pos = pos_start;
    accel = accel_start;
    update_pos_vel_accel(pos, vel, accel, dt, 0.0, -1.0, -1.0);
    EXPECT_FLOAT_EQ(vel, vel_start + accel * dt);
    EXPECT_FLOAT_EQ(pos, pos_start + vel_start * dt + 0.5 * accel * sq(dt));

    // test unlimited behaviour
    // zero error should result in normal behaviour
    // 5
    vel = vel_start;
    pos = pos_start;
    accel = accel_start;
    update_pos_vel_accel(pos, vel, accel, dt, 1.0, 0.0, 0.0);
    EXPECT_FLOAT_EQ(vel, vel_start + accel * dt);
    EXPECT_FLOAT_EQ(pos, pos_start + vel_start * dt + 0.5 * accel * sq(dt));

    // 6
    vel = vel_start;
    pos = pos_start;
    accel = -accel_start;
    update_pos_vel_accel(pos, vel, accel, dt, 1.0, 0.0, 0.0);
    EXPECT_FLOAT_EQ(vel, vel_start + accel * dt);
    EXPECT_FLOAT_EQ(pos, pos_start + vel_start * dt + 0.5 * accel * sq(dt));

    // 7
    vel = vel_start;
    pos = pos_start;
    accel = accel_start;
    update_pos_vel_accel(pos, vel, accel, dt, -1.0, 0.0, 0.0);
    EXPECT_FLOAT_EQ(vel, vel_start + accel * dt);
    EXPECT_FLOAT_EQ(pos, pos_start + vel_start * dt + 0.5 * accel * sq(dt));

    // 8
    vel = vel_start;
    pos = pos_start;
    accel = -accel_start;
    update_pos_vel_accel(pos, vel, accel, dt, -1.0, 0.0, 0.0);
    EXPECT_FLOAT_EQ(vel, vel_start + accel * dt);
    EXPECT_FLOAT_EQ(pos, pos_start + vel_start * dt + 0.5 * accel * sq(dt));
    
    // error sign opposite to limit sign should result in normal behaviour
    // 9
    vel = vel_start;
    pos = pos_start;
    accel = accel_start;
    update_pos_vel_accel(pos, vel, accel, dt, 1.0, -1.0, -1.0);
    EXPECT_FLOAT_EQ(vel, vel_start + accel * dt);
    EXPECT_FLOAT_EQ(pos, pos_start + vel_start * dt + 0.5 * accel * sq(dt));

    // 10
    vel = vel_start;
    pos = pos_start;
    accel = -accel_start;
    update_pos_vel_accel(pos, vel, accel, dt, 1.0, -1.0, -1.0);
    EXPECT_FLOAT_EQ(vel, vel_start + accel * dt);
    EXPECT_FLOAT_EQ(pos, pos_start + vel_start * dt + 0.5 * accel * sq(dt));

    // 11
    vel = vel_start;
    pos = pos_start;
    accel = accel_start;
    update_pos_vel_accel(pos, vel, accel, dt, -1.0, 1.0, 1.0);
    EXPECT_FLOAT_EQ(vel, vel_start + accel * dt);
    EXPECT_FLOAT_EQ(pos, pos_start + vel_start * dt + 0.5 * accel * sq(dt));

    // 12
    vel = vel_start;
    pos = pos_start;
    accel = -accel_start;
    update_pos_vel_accel(pos, vel, accel, dt, -1.0, 1.0, 1.0);
    EXPECT_FLOAT_EQ(vel, vel_start + accel * dt);
    EXPECT_FLOAT_EQ(pos, pos_start + vel_start * dt + 0.5 * accel * sq(dt));
    
    // error sign same as limit sign should result various limited behaviours
    // 13
    vel = vel_start;
    pos = pos_start;
    accel = accel_start;
    update_pos_vel_accel(pos, vel, accel, dt, 1.0, 1.0, 1.0);
    // vel is not increased
    EXPECT_FLOAT_EQ(vel, vel_start);
    // pos is not increased
    EXPECT_FLOAT_EQ(pos, pos_start);

    // 14
    vel = vel_start;
    pos = pos_start;
    accel = -accel_start;
    update_pos_vel_accel(pos, vel, accel, dt, 1.0, 1.0, 1.0);
    // vel is decreased
    EXPECT_FLOAT_EQ(vel, vel_start + accel * dt);
    // pos is not increased
    EXPECT_FLOAT_EQ(pos, pos_start);

    // 15
    vel = vel_start;
    pos = pos_start;
    accel = accel_start;
    update_pos_vel_accel(pos, vel, accel, dt, -1.0, -1.0, -1.0);
    // vel is increased
    EXPECT_FLOAT_EQ(vel, vel_start + accel * dt);
    // pos is increased
    EXPECT_FLOAT_EQ(pos, pos_start + vel_start * dt + 0.5 * accel * sq(dt));

    // 16
    vel = vel_start;
    pos = pos_start;
    accel = -accel_start;
    update_pos_vel_accel(pos, vel, accel, dt, -1.0, -1.0, -1.0);
    // velocity is limited but limit is not applied because velocity is reducing
    EXPECT_FLOAT_EQ(vel, vel_start + accel * dt);
    // pos is increased
    EXPECT_FLOAT_EQ(pos, pos_start + vel_start * dt + 0.5 * accel * sq(dt));

    // 17
    vel = -vel_start;
    pos = pos_start;
    accel = accel_start;
    update_pos_vel_accel(pos, vel, accel, dt, 1.0, 1.0, 1.0);
    // velocity is limited but limit is not applied because velocity is reducing
    EXPECT_FLOAT_EQ(vel, -vel_start + accel * dt);
    // pos is decreased
    EXPECT_FLOAT_EQ(pos, pos_start - vel_start * dt + 0.5 * accel * sq(dt));

    // 18
    vel_start = 0.1 * accel_start * dt;
    vel = vel_start;
    pos = pos_start;
    accel = -accel_start;
    update_pos_vel_accel(pos, vel, accel, dt, -1.0, -1.0, -1.0);
    // velocity is limited but limit is not applied because velocity is reducing
    // final result is zero because velocity would change sign during dt
    EXPECT_FLOAT_EQ(vel, 0.0);
    // pos is not changed because is_negative(vel_start * dt + 0.5 * accel * sq(t))
    EXPECT_FLOAT_EQ(pos, pos_start);

    // 19
    vel = -vel_start;
    pos = pos_start;
    accel = accel_start;
    update_pos_vel_accel(pos, vel, accel, dt, 1.0, 1.0, 1.0);
    // velocity is limited but limit is not applied because velocity is reducing
    // final result is zero because velocity would change sign during dt
    EXPECT_FLOAT_EQ(vel, 0.0);
    // pos is not changed because is_negative(vel_start * dt + 0.5 * accel * sq(t))
    EXPECT_FLOAT_EQ(pos, pos_start);


    // test for update_pos_vel_accel includes update_vel_accel.
    // test unlimited behaviour
    
    // 1
    pos_start = 17;
    vel_start = 20;
    accel_start = 1.0;
    Vector2p posxy = Vector2p(pos_start, 0.0);
    Vector2f velxy = Vector2f(vel_start, 0.0);
    Vector2f accelxy = Vector2f(accel_start, 0.0);
    Vector2f limit = Vector2f(0.0, 0.0);
    Vector2f pos_error = Vector2f(0.0, 0.0);
    Vector2f vel_error = Vector2f(0.0, 0.0);
    update_pos_vel_accel_xy(posxy, velxy, accelxy, dt, limit, pos_error, vel_error);
    EXPECT_FLOAT_EQ(velxy.x, vel_start + accelxy.x * dt);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
    EXPECT_FLOAT_EQ(posxy.x, pos_start + vel_start * dt + 0.5 * accelxy.x * sq(dt));
    EXPECT_FLOAT_EQ(velxy.y, 0.0);

    // 2
    posxy = Vector2p(pos_start, 0.0);
    velxy = Vector2f(vel_start, 0.0);
    accelxy = Vector2f(-accel_start, 0.0);
    limit = Vector2f(0.0, 0.0);
    pos_error = Vector2f(0.0, 0.0);
    vel_error = Vector2f(0.0, 0.0);
    update_pos_vel_accel_xy(posxy, velxy, accelxy, dt, limit, pos_error, vel_error);
    EXPECT_FLOAT_EQ(velxy.x, vel_start + accelxy.x * dt);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
    EXPECT_FLOAT_EQ(posxy.x, pos_start + vel_start * dt + 0.5 * accelxy.x * sq(dt));
    EXPECT_FLOAT_EQ(velxy.y, 0.0);

    // error has no impact when not limited
    // 3
    posxy = Vector2p(pos_start, 0.0);
    velxy = Vector2f(vel_start, 0.0);
    accelxy = Vector2f(accel_start, 0.0);
    limit = Vector2f(0.0, 0.0);
    pos_error = Vector2f(1.0, 0.0);
    vel_error = Vector2f(1.0, 0.0);
    update_pos_vel_accel_xy(posxy, velxy, accelxy, dt, limit, pos_error, vel_error);
    EXPECT_FLOAT_EQ(velxy.x, vel_start + accelxy.x * dt);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
    EXPECT_FLOAT_EQ(posxy.x, pos_start + vel_start * dt + 0.5 * accelxy.x * sq(dt));
    EXPECT_FLOAT_EQ(velxy.y, 0.0);

    // 4
    posxy = Vector2p(pos_start, 0.0);
    velxy = Vector2f(vel_start, 0.0);
    accelxy = Vector2f(accel_start, 0.0);
    limit = Vector2f(0.0, 0.0);
    pos_error = Vector2f(0.0, 0.0);
    vel_error = Vector2f(0.0, 0.0);
    update_pos_vel_accel_xy(posxy, velxy, accelxy, dt, limit, pos_error, vel_error);
    EXPECT_FLOAT_EQ(velxy.x, vel_start + accelxy.x * dt);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
    EXPECT_FLOAT_EQ(posxy.x, pos_start + vel_start * dt + 0.5 * accelxy.x * sq(dt));
    EXPECT_FLOAT_EQ(velxy.y, 0.0);

    // test unlimited behaviour
    // zero error should result in normal behaviour
    // 5
    posxy = Vector2p(pos_start, 0.0);
    velxy = Vector2f(vel_start, 0.0);
    accelxy = Vector2f(accel_start, 0.0);
    limit = Vector2f(1.0, 0.0);
    pos_error = Vector2f(0.0, 0.0);
    vel_error = Vector2f(0.0, 0.0);
    update_pos_vel_accel_xy(posxy, velxy, accelxy, dt, limit, pos_error, vel_error);
    EXPECT_FLOAT_EQ(velxy.x, vel_start + accelxy.x * dt);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
    EXPECT_FLOAT_EQ(posxy.x, pos_start + vel_start * dt + 0.5 * accelxy.x * sq(dt));
    EXPECT_FLOAT_EQ(velxy.y, 0.0);

    // 6
    posxy = Vector2p(pos_start, 0.0);
    velxy = Vector2f(vel_start, 0.0);
    accelxy = Vector2f(-accel_start, 0.0);
    limit = Vector2f(1.0, 0.0);
    pos_error = Vector2f(0.0, 0.0);
    vel_error = Vector2f(0.0, 0.0);
    update_pos_vel_accel_xy(posxy, velxy, accelxy, dt, limit, pos_error, vel_error);
    EXPECT_FLOAT_EQ(velxy.x, vel_start + accelxy.x * dt);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
    EXPECT_FLOAT_EQ(posxy.x, pos_start + vel_start * dt + 0.5 * accelxy.x * sq(dt));
    EXPECT_FLOAT_EQ(velxy.y, 0.0);

    // 7
    posxy = Vector2p(pos_start, 0.0);
    velxy = Vector2f(vel_start, 0.0);
    accelxy = Vector2f(accel_start, 0.0);
    limit = Vector2f(1.0, 0.0);
    pos_error = Vector2f(0.0, 0.0);
    vel_error = Vector2f(0.0, 0.0);
    update_pos_vel_accel_xy(posxy, velxy, accelxy, dt, limit, pos_error, vel_error);
    EXPECT_FLOAT_EQ(velxy.x, vel_start + accelxy.x * dt);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
    EXPECT_FLOAT_EQ(posxy.x, pos_start + vel_start * dt + 0.5 * accelxy.x * sq(dt));
    EXPECT_FLOAT_EQ(velxy.y, 0.0);

    // 8
    posxy = Vector2p(pos_start, 0.0);
    velxy = Vector2f(vel_start, 0.0);
    accelxy = Vector2f(-accel_start, 0.0);
    limit = Vector2f(1.0, 0.0);
    pos_error = Vector2f(0.0, 0.0);
    vel_error = Vector2f(0.0, 0.0);
    update_pos_vel_accel_xy(posxy, velxy, accelxy, dt, limit, pos_error, vel_error);
    EXPECT_FLOAT_EQ(velxy.x, vel_start + accelxy.x * dt);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
    EXPECT_FLOAT_EQ(posxy.x, pos_start + vel_start * dt + 0.5 * accelxy.x * sq(dt));
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
    
    // error sign opposite to limit sign should result in normal behaviour
    // 9
    posxy = Vector2p(pos_start, 0.0);
    velxy = Vector2f(vel_start, 0.0);
    accelxy = Vector2f(accel_start, 0.0);
    limit = Vector2f(1.0, 0.0);
    pos_error = Vector2f(-1.0, 0.0);
    vel_error = Vector2f(-1.0, 0.0);
    update_pos_vel_accel_xy(posxy, velxy, accelxy, dt, limit, pos_error, vel_error);
    EXPECT_FLOAT_EQ(velxy.x, vel_start + accelxy.x * dt);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
    EXPECT_FLOAT_EQ(posxy.x, pos_start + vel_start * dt + 0.5 * accelxy.x * sq(dt));
    EXPECT_FLOAT_EQ(velxy.y, 0.0);

    // 10
    posxy = Vector2p(pos_start, 0.0);
    velxy = Vector2f(vel_start, 0.0);
    accelxy = Vector2f(-accel_start, 0.0);
    limit = Vector2f(1.0, 0.0);
    pos_error = Vector2f(-1.0, 0.0);
    vel_error = Vector2f(-1.0, 0.0);
    update_pos_vel_accel_xy(posxy, velxy, accelxy, dt, limit, pos_error, vel_error);
    EXPECT_FLOAT_EQ(velxy.x, vel_start + accelxy.x * dt);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
    EXPECT_FLOAT_EQ(posxy.x, pos_start + vel_start * dt + 0.5 * accelxy.x * sq(dt));
    EXPECT_FLOAT_EQ(velxy.y, 0.0);

    // 11
    posxy = Vector2p(pos_start, 0.0);
    velxy = Vector2f(vel_start, 0.0);
    accelxy = Vector2f(accel_start, 0.0);
    limit = Vector2f(-1.0, 0.0);
    pos_error = Vector2f(1.0, 0.0);
    vel_error = Vector2f(1.0, 0.0);
    update_pos_vel_accel_xy(posxy, velxy, accelxy, dt, limit, pos_error, vel_error);
    EXPECT_FLOAT_EQ(velxy.x, vel_start + accelxy.x * dt);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
    EXPECT_FLOAT_EQ(posxy.x, pos_start + vel_start * dt + 0.5 * accelxy.x * sq(dt));
    EXPECT_FLOAT_EQ(velxy.y, 0.0);

    // 12
    posxy = Vector2p(pos_start, 0.0);
    velxy = Vector2f(vel_start, 0.0);
    accelxy = Vector2f(-accel_start, 0.0);
    limit = Vector2f(-1.0, 0.0);
    pos_error = Vector2f(1.0, 0.0);
    vel_error = Vector2f(1.0, 0.0);
    update_pos_vel_accel_xy(posxy, velxy, accelxy, dt, limit, pos_error, vel_error);
    EXPECT_FLOAT_EQ(velxy.x, vel_start + accelxy.x * dt);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
    EXPECT_FLOAT_EQ(posxy.x, pos_start + vel_start * dt + 0.5 * accelxy.x * sq(dt));
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
    
    // error sign same as limit sign should result various limited behaviours
    // 13
    posxy = Vector2p(pos_start, 0.0);
    velxy = Vector2f(vel_start, 0.0);
    accelxy = Vector2f(accel_start, 0.0);
    limit = Vector2f(1.0, 0.0);
    pos_error = Vector2f(1.0, 0.0);
    vel_error = Vector2f(1.0, 0.0);
    update_pos_vel_accel_xy(posxy, velxy, accelxy, dt, limit, pos_error, vel_error);
    // vel is not increased
    EXPECT_FLOAT_EQ(velxy.x, vel_start);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
    // pos is not increased
    EXPECT_FLOAT_EQ(posxy.x, pos_start);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);

    // 14
    posxy = Vector2p(pos_start, 0.0);
    velxy = Vector2f(vel_start, 0.0);
    accelxy = Vector2f(-accel_start, 0.0);
    limit = Vector2f(1.0, 0.0);
    pos_error = Vector2f(1.0, 0.0);
    vel_error = Vector2f(1.0, 0.0);
    update_pos_vel_accel_xy(posxy, velxy, accelxy, dt, limit, pos_error, vel_error);
    // vel is decreased
    EXPECT_FLOAT_EQ(velxy.x, vel_start + accelxy.x * dt);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
    // pos is not increased
    EXPECT_FLOAT_EQ(posxy.x, pos_start);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);

    // 15
    posxy = Vector2p(pos_start, 0.0);
    velxy = Vector2f(vel_start, 0.0);
    accelxy = Vector2f(accel_start, 0.0);
    limit = Vector2f(-1.0, 0.0);
    pos_error = Vector2f(-1.0, 0.0);
    vel_error = Vector2f(-1.0, 0.0);
    update_pos_vel_accel_xy(posxy, velxy, accelxy, dt, limit, pos_error, vel_error);
    // vel is increased
    EXPECT_FLOAT_EQ(velxy.x, vel_start + accelxy.x * dt);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
    // pos is increased
    EXPECT_FLOAT_EQ(posxy.x, pos_start + vel_start * dt + 0.5 * accelxy.x * sq(dt));
    EXPECT_FLOAT_EQ(velxy.y, 0.0);

    // 16
    posxy = Vector2p(pos_start, 0.0);
    velxy = Vector2f(vel_start, 0.0);
    accelxy = Vector2f(-accel_start, 0.0);
    limit = Vector2f(-1.0, 0.0);
    pos_error = Vector2f(-1.0, 0.0);
    vel_error = Vector2f(-1.0, 0.0);
    update_pos_vel_accel_xy(posxy, velxy, accelxy, dt, limit, pos_error, vel_error);
    // velocity is limited but limit is not applied because velocity is reducing
    EXPECT_FLOAT_EQ(velxy.x, vel_start + accelxy.x * dt);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
    // pos is increased
    EXPECT_FLOAT_EQ(posxy.x, pos_start + vel_start * dt + 0.5 * accelxy.x * sq(dt));
    EXPECT_FLOAT_EQ(velxy.y, 0.0);

    // 17
    posxy = Vector2p(pos_start, 0.0);
    velxy = Vector2f(-vel_start, 0.0);
    accelxy = Vector2f(accel_start, 0.0);
    limit = Vector2f(1.0, 0.0);
    pos_error = Vector2f(1.0, 0.0);
    vel_error = Vector2f(1.0, 0.0);
    update_pos_vel_accel_xy(posxy, velxy, accelxy, dt, limit, pos_error, vel_error);
    // velocity is limited but limit is not applied because velocity is reducing
    EXPECT_FLOAT_EQ(velxy.x, -vel_start + accelxy.x * dt);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
    // pos is decreased
    EXPECT_FLOAT_EQ(posxy.x, pos_start - vel_start * dt + 0.5 * accelxy.x * sq(dt));
    EXPECT_FLOAT_EQ(velxy.y, 0.0);

    // 18
    vel_start = 0.1 * accel_start * dt;
    posxy = Vector2p(pos_start, 0.0);
    velxy = Vector2f(vel_start, 0.0);
    accelxy = Vector2f(-accel_start, 0.0);
    limit = Vector2f(-1.0, 0.0);
    pos_error = Vector2f(-1.0, 0.0);
    vel_error = Vector2f(-1.0, 0.0);
    update_pos_vel_accel_xy(posxy, velxy, accelxy, dt, limit, pos_error, vel_error);
    // velocity is limited but limit is not applied because velocity is reducing
    // ideally this would be zero but code makes a simplification here
    EXPECT_FLOAT_EQ(velxy.x, vel_start + accelxy.x * dt);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
    // pos is not changed because is_negative(vel_start * dt + 0.5 * accel * sq(t))
    EXPECT_FLOAT_EQ(posxy.x, pos_start);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);

    // 19
    posxy = Vector2p(pos_start, 0.0);
    velxy = Vector2f(-vel_start, 0.0);
    accelxy = Vector2f(accel_start, 0.0);
    limit = Vector2f(1.0, 0.0);
    pos_error = Vector2f(1.0, 0.0);
    vel_error = Vector2f(1.0, 0.0);
    update_pos_vel_accel_xy(posxy, velxy, accelxy, dt, limit, pos_error, vel_error);
    // velocity is limited but limit is not applied because velocity is reducing
    // ideally this would be zero but code makes a simplification here
    EXPECT_FLOAT_EQ(velxy.x, -vel_start + accelxy.x * dt);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
    // pos is not changed because is_negative(vel_start * dt + 0.5 * accel * sq(t))
    EXPECT_FLOAT_EQ(posxy.x, pos_start);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
}

TEST(Control, AccelCorrectionPriorityXY)
{
    const float accel_max = 5.0f;

    // Correction is primary; a parallel trajectory receives only the remaining budget.
    EXPECT_EQ(allocate_accel_correction_priority(Vector2f(3.0f, 0.0f), Vector2f(4.0f, 0.0f), accel_max),
              Vector2f(5.0f, 0.0f));

    // Allocation is conservative for other directions: admitted trajectory
    // magnitude never exceeds the scalar budget left by correction.
    EXPECT_EQ(allocate_accel_correction_priority(Vector2f(3.0f, 0.0f), Vector2f(0.0f, 4.0f), accel_max),
              Vector2f(3.0f, 2.0f));
    EXPECT_EQ(allocate_accel_correction_priority(Vector2f(3.0f, 0.0f), Vector2f(-4.0f, 0.0f), accel_max),
              Vector2f(1.0f, 0.0f));

    // Unsaturated trajectory demand passes through unchanged.
    EXPECT_EQ(allocate_accel_correction_priority(Vector2f(1.0f, 0.0f), Vector2f(2.0f, 1.0f), accel_max),
              Vector2f(3.0f, 1.0f));
    EXPECT_EQ(allocate_accel_correction_priority(Vector2f(), Vector2f(3.0f, 4.0f), accel_max),
              Vector2f(3.0f, 4.0f));

    // Correction at or beyond the envelope receives full priority and all
    // trajectory acceleration is shed. The final controller limiter owns any
    // required correction saturation and anti-windup.
    EXPECT_EQ(allocate_accel_correction_priority(Vector2f(5.0f, 0.0f), Vector2f(0.0f, 4.0f), accel_max),
              Vector2f(5.0f, 0.0f));
    EXPECT_EQ(allocate_accel_correction_priority(Vector2f(6.0f, 0.0f), Vector2f(-4.0f, 0.0f), accel_max),
              Vector2f(6.0f, 0.0f));

    // A non-positive envelope cannot safely admit secondary acceleration.
    EXPECT_EQ(allocate_accel_correction_priority(Vector2f(1.0f, 2.0f), Vector2f(3.0f, 4.0f), 0.0f),
              Vector2f(1.0f, 2.0f));

    // Property coverage across relative directions and primary magnitudes.
    for (uint16_t correction_angle_deg = 0; correction_angle_deg < 360; correction_angle_deg += 15) {
        const float correction_angle_rad = radians(correction_angle_deg);
        for (uint16_t trajectory_angle_deg = 0; trajectory_angle_deg < 360; trajectory_angle_deg += 15) {
            const float trajectory_angle_rad = radians(trajectory_angle_deg);
            for (const float correction_magnitude : {0.0f, 1.0f, 4.9f, 5.0f, 6.0f}) {
                const Vector2f correction(cosf(correction_angle_rad) * correction_magnitude,
                                          sinf(correction_angle_rad) * correction_magnitude);
                const Vector2f trajectory(cosf(trajectory_angle_rad) * 10.0f,
                                          sinf(trajectory_angle_rad) * 10.0f);
                const Vector2f combined = allocate_accel_correction_priority(correction, trajectory, accel_max);

                if (correction_magnitude <= accel_max) {
                    EXPECT_LE(combined.length(), accel_max + 1.0e-5f);
                    EXPECT_LE((combined - correction).length(), accel_max - correction_magnitude + 1.0e-5f);
                } else {
                    EXPECT_EQ(combined, correction);
                }
            }
        }
    }
}

TEST(Control, AccelCorrectionPriorityAxis)
{
    constexpr float accel_min = -4.0f;
    constexpr float accel_max = 6.0f;

    EXPECT_FLOAT_EQ(allocate_accel_correction_priority(2.0f, 10.0f, accel_min, accel_max), 6.0f);
    EXPECT_FLOAT_EQ(allocate_accel_correction_priority(-2.0f, -10.0f, accel_min, accel_max), -4.0f);
    EXPECT_FLOAT_EQ(allocate_accel_correction_priority(2.0f, -1.0f, accel_min, accel_max), 1.0f);
    EXPECT_FLOAT_EQ(allocate_accel_correction_priority(0.0f, 3.0f, accel_min, accel_max), 3.0f);

    // At either correction bound no trajectory authority remains, even if the
    // requested trajectory points in the opposite direction.
    EXPECT_FLOAT_EQ(allocate_accel_correction_priority(6.0f, -10.0f, accel_min, accel_max), 6.0f);
    EXPECT_FLOAT_EQ(allocate_accel_correction_priority(-4.0f, 10.0f, accel_min, accel_max), -4.0f);

    // Out-of-envelope correction is never clipped by the allocator.
    EXPECT_FLOAT_EQ(allocate_accel_correction_priority(7.0f, -10.0f, accel_min, accel_max), 7.0f);
    EXPECT_FLOAT_EQ(allocate_accel_correction_priority(-5.0f, 10.0f, accel_min, accel_max), -5.0f);

    // Invalid bounds admit no trajectory acceleration.
    EXPECT_FLOAT_EQ(allocate_accel_correction_priority(2.0f, 10.0f, 0.0f, 0.0f), 2.0f);

    // Admitted trajectory authority decreases monotonically as correction
    // approaches either asymmetric limit.
    float previous_positive_admission = accel_max;
    for (float correction = 0.0f; correction <= accel_max; correction += 0.25f) {
        const float combined = allocate_accel_correction_priority(correction, 100.0f, accel_min, accel_max);
        const float admission = combined - correction;
        EXPECT_LE(admission, previous_positive_admission + 1.0e-6f);
        EXPECT_LE(combined, accel_max);
        EXPECT_GE(combined, accel_min);
        previous_positive_admission = admission;
    }

    float previous_negative_admission = -accel_min;
    for (float correction = 0.0f; correction >= accel_min; correction -= 0.25f) {
        const float combined = allocate_accel_correction_priority(correction, -100.0f, accel_min, accel_max);
        const float admission = fabsf(combined - correction);
        EXPECT_LE(admission, previous_negative_admission + 1.0e-6f);
        EXPECT_LE(combined, accel_max);
        EXPECT_GE(combined, accel_min);
        previous_negative_admission = admission;
    }
}

// catch floating point exceptions
sigjmp_buf avert_your_eyes_children;
static void _tc_sig_fpe(int signum)
{
    siglongjmp(avert_your_eyes_children, 1);
}

TEST(Control, test_limit_accel)
{
    // reproduction of FPE (https://github.com/ArduPilot/ardupilot/issues/28969)
    // FPE will only be raised in SITL HAL, so compiling for linux HAL
    // isn't useful.
    const Vector2f vel{
        0.984285712, 0.176583186
    };
    Vector2f accel{99.9008408, -557.304077};
    const float accel_max = 566.187256;

    struct sigaction old_sa_fpe = {};

    struct sigaction sa_fpe = {};
    sigemptyset(&sa_fpe.sa_mask);
    sa_fpe.sa_handler = _tc_sig_fpe;
    if (sigaction(SIGFPE, &sa_fpe, &old_sa_fpe) == -1) {
        abort();
    }
    const int excepts = FE_UNDERFLOW | FE_OVERFLOW | FE_INVALID;
    fexcept_t old_except_flags;
    if (fegetexceptflag(&old_except_flags, excepts) == -1) {
        abort();
    }

    feenableexcept(excepts);

    bool signal_caught = false;
    if (sigsetjmp(avert_your_eyes_children, 1)) {
        // we come through here if an FPE is triggered (via a goto in
        // our custom signal handler, _tc_sig_fpe)
        signal_caught = true;
    } else {
        // we come through here normally
        EXPECT_TRUE(limit_accel_xy(vel, accel, accel_max));
    }

    EXPECT_FALSE(signal_caught);

    // now restore the original fpe handling
    if (fesetexceptflag(&old_except_flags, excepts) == -1) {
        abort();
    }
    sigaction(SIGFPE, &old_sa_fpe, nullptr);
}

AP_GTEST_MAIN()
int hal = 0;
