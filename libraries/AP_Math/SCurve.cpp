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

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_InternalError/AP_InternalError.h>
#include "SCurve.h"

extern const AP_HAL::HAL &hal;

#define SEG_INIT        0
#define SEG_ACCEL_MAX   4
#define SEG_ACCEL_END   7
#define SEG_CHANGE_END  14
#define SEG_CONST       15
#define SEG_DECEL_END   22

// constructor
SCurve::SCurve()
{
    init();
}

// initialise and clear the path
void SCurve::init()
{
    jerk_time = 0.0f;
    jerk_max = 0.0f;
    accel_max = 0.0f;
    vel_max = 0.0f;
    time = 0.0f;
    num_segs = SEG_INIT;
    add_segment(num_segs, 0.0f, SegmentType::CONSTANT_JERK, 0.0f, 0.0f, 0.0f, 0.0f);
    track.zero();
    delta_unit.zero();
}

// set maximum velocity and re-calculate the path using these limits
void SCurve::set_speed_max(float speed_xy, float speed_up, float speed_down)
{
    // segment accelerations can not be changed after segment creation.
    const float track_speed_max = kinematic_limit(delta_unit, speed_xy, speed_up, fabsf(speed_down));

    if (is_equal(vel_max, track_speed_max)) {
        // new speed is equal to current speed maximum
        return;
    }

    if (is_zero(vel_max) || is_zero(track_speed_max)) {
        // new or original speeds are set to zero
        return;
    }
    vel_max = track_speed_max;

    // Check path has been defined
    if (num_segs != segments_max) {
        return;
    }

    if (time >= segment[SEG_CONST].end_time) {
        return;
    }

    // re-calculate the s-curve path based on update speeds

    const float Pend = segment[SEG_DECEL_END].end_pos;
    float Vend = MIN(vel_max, segment[SEG_DECEL_END].end_vel);
    uint8_t seg;

    if (is_zero(time)) {
        // Path has not started to we can recompute the path
        const float Vstart = MIN(vel_max, segment[SEG_INIT].end_vel);
        num_segs = SEG_INIT;
        add_segment(num_segs, 0.0f, SegmentType::CONSTANT_JERK, 0.0f, 0.0f, 0.0f, 0.0f);
        add_segments(Pend);
        set_origin_speed_max(Vstart);
        set_destination_speed_max(Vend);
        return;
    }

    if (time >= segment[SEG_ACCEL_END].end_time && time <= segment[SEG_CHANGE_END].end_time) {
        // In the change speed phase
        // Move adjust phase to acceleration phase to provide room for further speed adjustments

        // set initial segment to last acceleration segment
        segment[SEG_INIT].seg_type = SegmentType::CONSTANT_JERK;
        segment[SEG_INIT].jerk_ref = 0.0f;
        segment[SEG_INIT].end_time = segment[SEG_ACCEL_END].end_time;
        segment[SEG_INIT].end_accel = segment[SEG_ACCEL_END].end_accel;
        segment[SEG_INIT].end_vel = segment[SEG_ACCEL_END].end_vel;
        segment[SEG_INIT].end_pos = segment[SEG_ACCEL_END].end_pos;

        // move change segments to acceleration segments
        for (uint8_t i = SEG_INIT+1; i <= SEG_ACCEL_END; i++) {
            segment[i].seg_type = segment[i+7].seg_type;
            segment[i].jerk_ref = segment[i+7].jerk_ref;
            segment[i].end_time = segment[i+7].end_time;
            segment[i].end_accel = segment[i+7].end_accel;
            segment[i].end_vel = segment[i+7].end_vel;
            segment[i].end_pos = segment[i+7].end_pos;
        }

        // set change segments to last acceleration speed
        for (uint8_t i = SEG_ACCEL_END+1; i <= SEG_CHANGE_END; i++) {
            segment[i].seg_type = SegmentType::CONSTANT_JERK;
            segment[i].jerk_ref = 0.0f;
            segment[i].end_time = segment[SEG_ACCEL_END].end_time;
            segment[i].end_accel = 0.0f;
            segment[i].end_vel = segment[SEG_ACCEL_END].end_vel;
            segment[i].end_pos = segment[SEG_ACCEL_END].end_pos;
        }

    } else if (time >= segment[SEG_CHANGE_END].end_time && time <= segment[SEG_CONST].end_time) {
        // In the constant speed phase
        // Move adjust phase to acceleration phase to provide room for further speed adjustments

        // set initial segment to last acceleration segment
        segment[SEG_INIT].seg_type = SegmentType::CONSTANT_JERK;
        segment[SEG_INIT].jerk_ref = 0.0f;
        segment[SEG_INIT].end_time = segment[SEG_CHANGE_END].end_time;
        segment[SEG_INIT].end_accel = 0.0f;
        segment[SEG_INIT].end_vel = segment[SEG_CHANGE_END].end_vel;
        segment[SEG_INIT].end_pos = segment[SEG_CHANGE_END].end_pos;

        // set acceleration and change segments to current constant speed
        float Jt_out, At_out, Vt_out, Pt_out;
        update(time, Jt_out, At_out, Vt_out, Pt_out);
        for (uint8_t i = SEG_INIT+1; i <= SEG_CHANGE_END; i++) {
            segment[i].seg_type = SegmentType::CONSTANT_JERK;
            segment[i].jerk_ref = 0.0f;
            segment[i].end_time = time;
            segment[i].end_accel = 0.0f;
            segment[i].end_vel = Vt_out;
            segment[i].end_pos = Pt_out;
        }
    }

    // Adjust the INIT and ACCEL segments for new speed
    if ((time <= segment[SEG_ACCEL_MAX].end_time) && is_positive(segment[SEG_ACCEL_MAX].end_time - segment[SEG_ACCEL_MAX-1].end_time) && (vel_max < segment[SEG_ACCEL_END].end_vel) && is_positive(segment[SEG_ACCEL_MAX].end_accel) ) {
        // Path has not finished constant positive acceleration segment
        // Reduce velocity as close to target velocity as possible

        float Vstart = segment[SEG_INIT].end_vel;

        // minimum velocity that can be obtained by shortening SEG_ACCEL_MAX
        float Vmin = segment[SEG_ACCEL_END].end_vel - segment[SEG_ACCEL_MAX].end_accel * (segment[SEG_ACCEL_MAX].end_time - MAX(time, segment[SEG_ACCEL_MAX-1].end_time));

        seg = SEG_INIT+1;

        float Jm, t2, t4, t6;
        calculate_path(jerk_time, jerk_max, Vstart, accel_max, MAX(Vmin, vel_max), Pend / 2.0f, Jm, t2, t4, t6);

        add_segments_jerk(seg, jerk_time, Jm, t2);
        add_segment_const_jerk(seg, t4, 0.0f);
        add_segments_jerk(seg, jerk_time, -Jm, t6);

        // add empty speed adjust segments
        for (uint8_t i = SEG_ACCEL_END+1; i <= SEG_CONST; i++) {
            segment[i].seg_type = SegmentType::CONSTANT_JERK;
            segment[i].jerk_ref = 0.0f;
            segment[i].end_time = segment[SEG_ACCEL_END].end_time;
            segment[i].end_accel = 0.0f;
            segment[i].end_vel = segment[SEG_ACCEL_END].end_vel;
            segment[i].end_pos = segment[SEG_ACCEL_END].end_pos;
        }

        calculate_path(jerk_time, jerk_max, 0.0f, accel_max, MAX(Vmin, vel_max), Pend / 2.0f, Jm, t2, t4, t6);

        seg = SEG_CONST + 1;
        add_segments_jerk(seg, jerk_time, -Jm, t6);
        add_segment_const_jerk(seg, t4, 0.0f);
        add_segments_jerk(seg, jerk_time, Jm, t2);

        // add to constant velocity segment to end at the correct position
        float dP = (Pend - segment[SEG_DECEL_END].end_pos);
        float t15 =  dP / segment[SEG_CONST].end_vel;
        for (uint8_t i = SEG_CONST; i <= SEG_DECEL_END; i++) {
            segment[i].end_time += t15;
            segment[i].end_pos += dP;
        }
    }

    // Adjust the CHANGE segments for new speed
    // start with empty speed adjust segments
    for (uint8_t i = SEG_ACCEL_END+1; i <= SEG_CHANGE_END; i++) {
        segment[i].seg_type = SegmentType::CONSTANT_JERK;
        segment[i].jerk_ref = 0.0f;
        segment[i].end_time = segment[SEG_ACCEL_END].end_time;
        segment[i].end_accel = 0.0f;
        segment[i].end_vel = segment[SEG_ACCEL_END].end_vel;
        segment[i].end_pos = segment[SEG_ACCEL_END].end_pos;
    }
    if (!is_equal(vel_max, segment[SEG_ACCEL_END].end_vel)) {
        // add velocity adjustment
        // check there is enough time to make velocity change
        // we use the approximation that the time will be distance/max_vel and 8 jerk segments
        float L = segment[SEG_CONST].end_pos - segment[SEG_ACCEL_END].end_pos;
        float Jm = 0;
        float t2 = 0;
        float t4 = 0;
        float t6 = 0;
        if ((vel_max < segment[SEG_ACCEL_END].end_vel) && (jerk_time*12.0f < L/segment[SEG_ACCEL_END].end_vel)) {
            // we have a problem here with small segments.
            calculate_path(jerk_time, jerk_max, vel_max, accel_max, segment[SEG_ACCEL_END].end_vel, L / 2.0f, Jm, t6, t4, t2);
            Jm = -Jm;

        } else if ((vel_max > segment[SEG_ACCEL_END].end_vel) && (L/(jerk_time*12.0f) > segment[SEG_ACCEL_END].end_vel)) {
            float Vm = MIN(vel_max, L/(jerk_time*12.0f));
            calculate_path(jerk_time, jerk_max, segment[SEG_ACCEL_END].end_vel, accel_max, Vm, L / 2.0f, Jm, t2, t4, t6);
        }

        seg = SEG_ACCEL_END + 1;
        if (!is_zero(Jm) && !is_negative(t2) && !is_negative(t4) && !is_negative(t6)) {
            add_segments_jerk(seg, jerk_time, Jm, t2);
            add_segment_const_jerk(seg, t4, 0.0f);
            add_segments_jerk(seg, jerk_time, -Jm, t6);
        }
    }

    // add deceleration segments
    // Earlier check should ensure that we should always have sufficient time to stop
    seg = SEG_CONST;
    Vend = MIN(Vend, segment[SEG_CHANGE_END].end_vel);
    add_segment_const_jerk(seg, 0.0f, 0.0f);
    if (Vend < segment[SEG_CHANGE_END].end_vel) {
        float Jm, t2, t4, t6;
        calculate_path(jerk_time, jerk_max, Vend, accel_max, segment[SEG_CONST].end_vel, Pend - segment[SEG_CONST].end_pos, Jm, t2, t4, t6);
        add_segments_jerk(seg, jerk_time, -Jm, t6);
        add_segment_const_jerk(seg, t4, 0.0f);
        add_segments_jerk(seg, jerk_time, Jm, t2);
    } else {
        // No deceleration is required
        for (uint8_t i = SEG_CONST+1; i <= SEG_DECEL_END; i++) {
            segment[i].seg_type = SegmentType::CONSTANT_JERK;
            segment[i].jerk_ref = 0.0f;
            segment[i].end_time = segment[SEG_CONST].end_time;
            segment[i].end_accel = 0.0f;
            segment[i].end_vel = segment[SEG_CONST].end_vel;
            segment[i].end_pos = segment[SEG_CONST].end_pos;
        }
    }

    // add to constant velocity segment to end at the correct position
    float dP = (Pend - segment[SEG_DECEL_END].end_pos);
    float t15 =  dP / segment[SEG_CONST].end_vel;
    for (uint8_t i = SEG_CONST; i <= SEG_DECEL_END; i++) {
        segment[i].end_time += t15;
        segment[i].end_pos += dP;
    }
}

// generate a trigonometric track in 3D space that moves over a straight line
// between two points defined by the origin and destination
void SCurve::calculate_track(const Vector3f &origin, const Vector3f &destination,
                             float speed_xy, float speed_up, float speed_down,
                             float accel_xy, float accel_z,
                             float jerk_time_sec, float jerk_maximum)
{
    init();

    // set jerk time and jerk max
    jerk_time = jerk_time_sec;
    jerk_max = jerk_maximum;

    // update speed and acceleration limits along path
    set_kinematic_limits(origin, destination,
                         speed_xy, speed_up, speed_down,
                         accel_xy, accel_z);

    // avoid divide-by zeros. Path will be left as a zero length path
    if (!is_positive(jerk_time) || !is_positive(jerk_max) || !is_positive(accel_max) || !is_positive(vel_max)) {
        return;
    }

    track = destination - origin;
    const float track_length = track.length();
    if (is_zero(track_length)) {
        // avoid possible divide by zero
        delta_unit.zero();
    } else {
        delta_unit = track.normalized();
        add_segments(track_length);
    }
}

// set the maximum vehicle speed at the origin
// returns the expected speed at the origin (will always be equal or lower to speed_cm)
float SCurve::set_origin_speed_max(float speed)
{
    // if path is zero length then start speed must be zero
    if (num_segs != segments_max) {
        return 0.0f;
    }

    // check speed is zero or positive
    // avoid re-calculating if unnecessary
    if (is_equal(segment[SEG_INIT].end_vel, speed)) {
        return speed;
    }

    float Vm = segment[SEG_ACCEL_END].end_vel;
    float L = segment[SEG_DECEL_END].end_pos;
    speed = MIN(speed, Vm);

    float Jm, t2, t4, t6;
    calculate_path(jerk_time, jerk_max, speed, accel_max, Vm, L / 2.0f, Jm, t2, t4, t6);

    uint8_t seg = SEG_INIT;
    add_segment(seg, 0.0f, SegmentType::CONSTANT_JERK, 0.0f, 0.0f, speed, 0.0f);
    add_segments_jerk(seg, jerk_time, Jm, t2);
    add_segment_const_jerk(seg, t4, 0.0f);
    add_segments_jerk(seg, jerk_time, -Jm, t6);

    // add empty speed change segments and constant speed segment
    for (uint8_t i = SEG_ACCEL_END+1; i <= SEG_CHANGE_END; i++) {
        segment[i].seg_type = SegmentType::CONSTANT_JERK;
        segment[i].jerk_ref = 0.0f;
        segment[i].end_time = segment[SEG_ACCEL_END].end_time;
        segment[i].end_accel = 0.0f;
        segment[i].end_vel = segment[SEG_ACCEL_END].end_vel;
        segment[i].end_pos = segment[SEG_ACCEL_END].end_pos;
    }

    calculate_path(jerk_time, jerk_max, 0.0f, accel_max, Vm, L - segment[SEG_CONST].end_pos, Jm, t2, t4, t6);

    seg = SEG_CONST;
    add_segment_const_jerk(seg, 0.0f, 0.0f);

    add_segments_jerk(seg, jerk_time, -Jm, t6);
    add_segment_const_jerk(seg, t4, 0.0f);
    add_segments_jerk(seg, jerk_time, Jm, t2);

    // add to constant velocity segment to end at the correct position
    float dP = (L - segment[SEG_DECEL_END].end_pos);
    float t15 =  dP / segment[SEG_CONST].end_vel;
    for (uint8_t i = SEG_CONST; i <= SEG_DECEL_END; i++) {
        segment[i].end_time += t15;
        segment[i].end_pos += dP;
    }
    return speed;
}

// set the maximum vehicle speed at the destination
void SCurve::set_destination_speed_max(float speed)
{
    // if path is zero length then start speed must be zero
    if (num_segs != segments_max) {
        return;
    }

    // avoid re-calculating if unnecessary
    if (is_equal(segment[segments_max-1].end_vel, speed)) {
        return;
    }

    float Vm = segment[SEG_CONST].end_vel;
    float L = segment[SEG_DECEL_END].end_pos;
    speed = MIN(speed, Vm);

    float Jm, t2, t4, t6;
    calculate_path(jerk_time, jerk_max, speed, accel_max, Vm, L / 2.0f, Jm, t2, t4, t6);

    uint8_t seg = SEG_CONST;
    add_segment_const_jerk(seg, 0.0f, 0.0f);

    add_segments_jerk(seg, jerk_time, -Jm, t6);
    add_segment_const_jerk(seg, t4, 0.0f);
    add_segments_jerk(seg, jerk_time, Jm, t2);

    // add to constant velocity segment to end at the correct position
    float dP = (L - segment[SEG_DECEL_END].end_pos);
    float t15 =  dP / segment[SEG_CONST].end_vel;
    for (uint8_t i = SEG_CONST; i <= SEG_DECEL_END; i++) {
        segment[i].end_time += t15;
        segment[i].end_pos += dP;
    }
}

// move target location along path from origin to destination
// prev_leg and next_leg are the paths before and after this path
// wp_radius is max distance from the waypoint at the apex of the turn
// fast_waypoint should be true if vehicle will not stop at end of this leg
// dt is the time increment the vehicle will move along the path
// target_pos should be set to this segment's origin and it will be updated to the current position target
// target_vel and target_accel are updated with new targets
// returns true if vehicle has passed the apex of the corner
bool SCurve::advance_target_along_track(SCurve &prev_leg, SCurve &next_leg, float wp_radius, bool fast_waypoint, float dt, Vector3f &target_pos, Vector3f &target_vel, Vector3f &target_accel)
{
    prev_leg.move_to_pos_vel_accel(dt, target_pos, target_vel, target_accel);
    move_from_pos_vel_accel(dt, target_pos, target_vel, target_accel);
    bool s_finished = finished();

    // check for change of leg on fast waypoint
    const float time_to_destination = get_time_remaining();
    if (fast_waypoint && braking() && is_zero(next_leg.get_time_elapsed()) && (time_to_destination <= next_leg.get_accel_finished_time())) {
        Vector3f turn_pos = -get_track();
        Vector3f turn_vel, turn_accel;
        move_from_time_pos_vel_accel(get_time_elapsed() + time_to_destination / 2.0f, turn_pos, turn_vel, turn_accel);
        next_leg.move_from_time_pos_vel_accel(time_to_destination / 2.0f, turn_pos, turn_vel, turn_accel);
        const float speed_min = MIN(get_speed_along_track(), next_leg.get_speed_along_track());
        const float accel_min = MIN(get_accel_along_track(), next_leg.get_accel_along_track());
        if ((get_time_remaining() < next_leg.time_end() / 2.0f) && (turn_pos.length() < wp_radius) &&
             (Vector2f(turn_vel.x, turn_vel.y).length() < speed_min) &&
             (Vector2f(turn_accel.x, turn_accel.y).length() < 2.0f*accel_min)) {
            next_leg.move_from_pos_vel_accel(dt, target_pos, target_vel, target_accel);
        }
    } else if (!is_zero(next_leg.get_time_elapsed())) {
        next_leg.move_from_pos_vel_accel(dt, target_pos, target_vel, target_accel);
        if (next_leg.get_time_elapsed() >= get_time_remaining()) {
            s_finished = true;
        }
    }

    return s_finished;
}

// increment time pointer and return the position, velocity and acceleration vectors relative to the origin
void SCurve::move_from_pos_vel_accel(float dt, Vector3f &pos, Vector3f &vel, Vector3f &accel)
{
    advance_time(dt);
    move_from_time_pos_vel_accel(time, pos, vel, accel);
}

// increment time pointer and return the position, velocity and acceleration vectors relative to the destination
void SCurve::move_to_pos_vel_accel(float dt, Vector3f &pos, Vector3f &vel, Vector3f &accel)
{
    advance_time(dt);
    move_from_time_pos_vel_accel(time, pos, vel, accel);
    pos -= track;
}

// return the position, velocity and acceleration vectors relative to the origin at a specified time along the path
void SCurve::move_from_time_pos_vel_accel(float time_now, Vector3f &pos, Vector3f &vel, Vector3f &accel)
{
    float scurve_P1 = 0.0f;
    float scurve_V1, scurve_A1, scurve_J1;
    update(time_now, scurve_J1, scurve_A1, scurve_V1, scurve_P1);
    pos += delta_unit * scurve_P1;
    vel += delta_unit * scurve_V1;
    accel += delta_unit * scurve_A1;
}

// time has reached the end of the sequence
bool SCurve::finished() const
{
    if (num_segs != segments_max) {
        return true;
    }
    return time > time_end();
}

// magnitude of the position vector at the end of the sequence
float SCurve::pos_end() const
{
    if (num_segs != segments_max) {
        return 0.0f;
    }
    return segment[SEG_DECEL_END].end_pos;
}

// time at the end of the sequence
float SCurve::time_end() const
{
    if (num_segs != segments_max) {
        return 0.0f;
    }
    return segment[SEG_DECEL_END].end_time;
}

// time left before sequence will complete
float SCurve::get_time_remaining() const
{
    if (num_segs != segments_max) {
        return 0.0f;
    }
    return segment[SEG_DECEL_END].end_time - time;
}

// time when acceleration section of the sequence will complete
float SCurve::get_accel_finished_time() const
{
    if (num_segs != segments_max) {
        return 0.0f;
    }
    return segment[SEG_ACCEL_END].end_time;
}

// return true if the sequence is braking to a stop
bool SCurve::braking() const
{
    if (num_segs != segments_max) {
        return true;
    }
    return time >= segment[SEG_CONST].end_time;
}

// calculate the jerk, acceleration, velocity and position at the provided time
void SCurve::update(float time_now, float &Jt_out, float &At_out, float &Vt_out, float &Pt_out)
{
    SegmentType Jtype;
    uint8_t pnt = num_segs;
    float tj;
    float Jm, T0, A0, V0, P0;

    // find active segment at time_now
    for (uint8_t i = 0; i < num_segs; i++) {
        if (time_now < segment[num_segs - 1 - i].end_time) {
            pnt = num_segs - 1 - i;
        }
    }
    if (pnt == 0) {
        Jtype = SegmentType::CONSTANT_JERK;
        Jm = 0.0f;
        T0 = segment[pnt].end_time;
        A0 = segment[pnt].end_accel;
        V0 = segment[pnt].end_vel;
        P0 = segment[pnt].end_pos;
    } else if (pnt == num_segs) {
        Jtype = SegmentType::CONSTANT_JERK;
        Jm = 0.0f;
        T0 = segment[pnt - 1].end_time;
        A0 = segment[pnt - 1].end_accel;
        V0 = segment[pnt - 1].end_vel;
        P0 = segment[pnt - 1].end_pos;
    } else {
        Jtype = segment[pnt].seg_type;
        Jm = segment[pnt].jerk_ref;
        tj = segment[pnt].end_time - segment[pnt - 1].end_time;
        T0 = segment[pnt - 1].end_time;
        A0 = segment[pnt - 1].end_accel;
        V0 = segment[pnt - 1].end_vel;
        P0 = segment[pnt - 1].end_pos;
    }

    switch (Jtype) {
    case SegmentType::CONSTANT_JERK:
        calc_javp_for_segment_const_jerk(time_now - T0, Jm, A0, V0, P0, Jt_out, At_out, Vt_out, Pt_out);
        break;
    case SegmentType::POSITIVE_JERK:
        calc_javp_for_segment_incr_jerk(time_now - T0, tj, Jm, A0, V0, P0, Jt_out, At_out, Vt_out, Pt_out);
        break;
    case SegmentType::NEGATIVE_JERK:
        calc_javp_for_segment_decr_jerk(time_now - T0, tj, Jm, A0, V0, P0, Jt_out, At_out, Vt_out, Pt_out);
        break;
    }
}

// calculate the jerk, acceleration, velocity and position at time time_now when running the constant jerk time segment
void SCurve::calc_javp_for_segment_const_jerk(float time_now, float J0, float A0, float V0, float P0, float &Jt, float &At, float &Vt, float &Pt) const
{
    Jt = J0;
    At = A0 + J0 * time_now;
    Vt = V0 + A0 * time_now + 0.5f * J0 * (time_now * time_now);
    Pt = P0 + V0 * time_now + 0.5f * A0 * (time_now * time_now) + (1.0f / 6.0f) * J0 * (time_now * time_now * time_now);
}

// Calculate the jerk, acceleration, velocity and position at time time_now when running the increasing jerk magnitude time segment based on a raised cosine profile
void SCurve::calc_javp_for_segment_incr_jerk(float time_now, float tj, float Jm, float A0, float V0, float P0, float &Jt, float &At, float &Vt, float &Pt) const
{
    float Alpha = Jm / 2.0f;
    float Beta = M_PI / tj;
    Jt = Alpha * (1.0f - cosf(Beta * time_now));
    At = A0 + Alpha * time_now - (Alpha / Beta) * sinf(Beta * time_now);
    Vt = V0 + A0 * time_now + (Alpha / 2.0f) * (time_now * time_now) + (Alpha / (Beta * Beta)) * cosf(Beta * time_now) - Alpha / (Beta * Beta);
    Pt = P0 + V0 * time_now + 0.5f * A0 * (time_now * time_now) + (-Alpha / (Beta * Beta)) * time_now + Alpha * (time_now * time_now * time_now) / 6.0f + (Alpha / (Beta * Beta * Beta)) * sinf(Beta * time_now);
}

// Calculate the jerk, acceleration, velocity and position at time time_now when running the decreasing jerk magnitude time segment based on a raised cosine profile
void SCurve::calc_javp_for_segment_decr_jerk(float time_now, float tj, float Jm, float A0, float V0, float P0, float &Jt, float &At, float &Vt, float &Pt) const
{
    float Alpha = Jm / 2.0f;
    float Beta = M_PI / tj;
    float AT = Alpha * tj;
    float VT = Alpha * ((tj * tj) / 2.0f - 2.0f / (Beta * Beta));
    float PT = Alpha * ((-1.0f / (Beta * Beta)) * tj + (1.0f / 6.0f) * (tj * tj * tj));
    Jt = Alpha * (1.0f - cosf(Beta * (time_now + tj)));
    At = (A0 - AT) + Alpha * (time_now + tj) - (Alpha / Beta) * sinf(Beta * (time_now + tj));
    Vt = (V0 - VT) + (A0 - AT) * time_now + 0.5f * Alpha * (time_now + tj) * (time_now + tj) + (Alpha / (Beta * Beta)) * cosf(Beta * (time_now + tj)) - Alpha / (Beta * Beta);
    Pt = (P0 - PT) + (V0 - VT) * time_now + 0.5f * (A0 - AT) * (time_now * time_now) + (-Alpha / (Beta * Beta)) * (time_now + tj) + (Alpha / 6.0f) * (time_now + tj) * (time_now + tj) * (time_now + tj) + (Alpha / (Beta * Beta * Beta)) * sinf(Beta * (time_now + tj));
}

// generate the segments for a path of length L
// the path consists of 23 segments
// 1 initial segment
// 7 segments forming the acceleration S-Curve
// 7 segments forming the velocity change S-Curve
// 1 constant velocity S-Curve
// 7 segments forming the deceleration S-Curve
void SCurve::add_segments(float L)
{
    if (is_zero(L)) {
        return;
    }

    float Jm, t2, t4, t6;
    calculate_path(jerk_time, jerk_max, 0.0f, accel_max, vel_max, L / 2.0f, Jm, t2, t4, t6);

    add_segments_jerk(num_segs, jerk_time, Jm, t2);
    add_segment_const_jerk(num_segs, t4, 0.0f);
    add_segments_jerk(num_segs, jerk_time, -Jm, t6);

    // add empty speed adjust segments
    add_segment_const_jerk(num_segs, 0.0f, 0.0f);
    add_segment_const_jerk(num_segs, 0.0f, 0.0f);
    add_segment_const_jerk(num_segs, 0.0f, 0.0f);
    add_segment_const_jerk(num_segs, 0.0f, 0.0f);
    add_segment_const_jerk(num_segs, 0.0f, 0.0f);
    add_segment_const_jerk(num_segs, 0.0f, 0.0f);
    add_segment_const_jerk(num_segs, 0.0f, 0.0f);

    float t15 = 2.0f * (L / 2.0f - segment[SEG_CHANGE_END].end_pos) / segment[SEG_CHANGE_END].end_vel;
    add_segment_const_jerk(num_segs, t15, 0.0f);

    add_segments_jerk(num_segs, jerk_time, -Jm, t6);
    add_segment_const_jerk(num_segs, t4, 0.0f);
    add_segments_jerk(num_segs, jerk_time, Jm, t2);
}

// calculate the segment times for the trigonometric S-Curve path defined by:
// tj - duration of the raised cosine jerk profile
// Jm - maximum value of the raised cosine jerk profile
// V0 - initial velocity magnitude
// Am - maximum constant acceleration
// Vm - maximum constant velocity
// L - Length of the path
// t2_out, t4_out, t6_out are the segment durations needed to achieve the kinimatic path specified by the input variables
void SCurve::calculate_path(float tj, float Jm, float V0, float Am, float Vm, float L, float &Jm_out, float &t2_out, float &t4_out, float &t6_out) const
{
    // init outputs
    Jm_out = 0.0f;
    t2_out = 0.0f;
    t4_out = 0.0f;
    t6_out = 0.0f;

    // check for invalid arguments
    if (!is_positive(tj) || !is_positive(Jm) || !is_positive(Am) || !is_positive(Vm) || !is_positive(L)) {
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arguments);
        return;
    }

    if (V0 >= Vm) {
        // no velocity change so all segments as zero length
        return;
    }

    Am = MIN(MIN(Am, (Vm - V0) / (2.0f * tj)), (L + 4.0f * V0 * tj) / (4.0f * sq(tj)));
    if (fabsf(Am) < Jm * tj) {
        Jm = Am / tj;
        if ((Vm <= V0 + 2.0f * Am * tj) || (L <= 4.0f * V0 * tj + 4.0f * Am * sq(tj))) {
            // solution = 0 - t6 t4 t2 = 0 0 0
            t2_out = 0.0f;
            t4_out = 0.0f;
            t6_out = 0.0f;
        } else {
            // solution = 2 - t6 t4 t2 = 0 1 0
            t2_out = 0.0f;
            t4_out = MIN(-(V0 - Vm + Am * tj + (Am * Am) / Jm) / Am, MAX(((Am * Am) * (-3.0f / 2.0f) + safe_sqrt((Am * Am * Am * Am) * (1.0f / 4.0f) + (Jm * Jm) * (V0 * V0) + (Am * Am) * (Jm * Jm) * (tj * tj) * (1.0f / 4.0f) + Am * (Jm * Jm) * L * 2.0f - (Am * Am) * Jm * V0 + (Am * Am * Am) * Jm * tj * (1.0f / 2.0f) - Am * (Jm * Jm) * V0 * tj) - Jm * V0 - Am * Jm * tj * (3.0f / 2.0f)) / (Am * Jm), ((Am * Am) * (-3.0f / 2.0f) - safe_sqrt((Am * Am * Am * Am) * (1.0f / 4.0f) + (Jm * Jm) * (V0 * V0) + (Am * Am) * (Jm * Jm) * (tj * tj) * (1.0f / 4.0f) + Am * (Jm * Jm) * L * 2.0f - (Am * Am) * Jm * V0 + (Am * Am * Am) * Jm * tj * (1.0f / 2.0f) - Am * (Jm * Jm) * V0 * tj) - Jm * V0 - Am * Jm * tj * (3.0f / 2.0f)) / (Am * Jm)));
            t6_out = 0.0f;
        }
    } else {
        if ((Vm < V0 + Am * tj + (Am * Am) / Jm) || (L < 1.0f / (Jm * Jm) * (Am * Am * Am + Am * Jm * (V0 * 2.0f + Am * tj * 2.0f)) + V0 * tj * 2.0f + Am * (tj * tj))) {
            // solution = 5 - t6 t4 t2 = 1 0 1
            Am = MIN(MIN(Am, MAX(Jm * (tj + safe_sqrt((V0 * -4.0f + Vm * 4.0f + Jm * (tj * tj)) / Jm)) * (-1.0f / 2.0f), Jm * (tj - safe_sqrt((V0 * -4.0f + Vm * 4.0f + Jm * (tj * tj)) / Jm)) * (-1.0f / 2.0f))), Jm * tj * (-2.0f / 3.0f) + ((Jm * Jm) * (tj * tj) * (1.0f / 9.0f) - Jm * V0 * (2.0f / 3.0f)) * 1.0f / powf(safe_sqrt(powf(- (Jm * Jm) * L * (1.0f / 2.0f) + (Jm * Jm * Jm) * (tj * tj * tj) * (8.0f / 2.7E1f) - Jm * tj * ((Jm * Jm) * (tj * tj) + Jm * V0 * 2.0f) * (1.0f / 3.0f) + (Jm * Jm) * V0 * tj, 2.0f) - powf((Jm * Jm) * (tj * tj) * (1.0f / 9.0f) - Jm * V0 * (2.0f / 3.0f), 3.0f)) + (Jm * Jm) * L * (1.0f / 2.0f) - (Jm * Jm * Jm) * (tj * tj * tj) * (8.0f / 2.7E1f) + Jm * tj * ((Jm * Jm) * (tj * tj) + Jm * V0 * 2.0f) * (1.0f / 3.0f) - (Jm * Jm) * V0 * tj, 1.0f / 3.0f) + powf(safe_sqrt(powf(- (Jm * Jm) * L * (1.0f / 2.0f) + (Jm * Jm * Jm) * (tj * tj * tj) * (8.0f / 2.7E1f) - Jm * tj * ((Jm * Jm) * (tj * tj) + Jm * V0 * 2.0f) * (1.0f / 3.0f) + (Jm * Jm) * V0 * tj, 2.0f) - powf((Jm * Jm) * (tj * tj) * (1.0f / 9.0f) - Jm * V0 * (2.0f / 3.0f), 3.0f)) + (Jm * Jm) * L * (1.0f / 2.0f) - (Jm * Jm * Jm) * (tj * tj * tj) * (8.0f / 2.7E1f) + Jm * tj * ((Jm * Jm) * (tj * tj) + Jm * V0 * 2.0f) * (1.0f / 3.0f) - (Jm * Jm) * V0 * tj, 1.0f / 3.0f));
            t2_out = Am / Jm - tj;
            t4_out = 0.0f;
            t6_out = t2_out;
        } else {
            // solution = 7 - t6 t4 t2 = 1 1 1
            t2_out = Am / Jm - tj;
            t4_out = MIN(-(V0 - Vm + Am * tj + (Am * Am) / Jm) / Am, MAX(((Am * Am) * (-3.0f / 2.0f) + safe_sqrt((Am * Am * Am * Am) * (1.0f / 4.0f) + (Jm * Jm) * (V0 * V0) + (Am * Am) * (Jm * Jm) * (tj * tj) * (1.0f / 4.0f) + Am * (Jm * Jm) * L * 2.0f - (Am * Am) * Jm * V0 + (Am * Am * Am) * Jm * tj * (1.0f / 2.0f) - Am * (Jm * Jm) * V0 * tj) - Jm * V0 - Am * Jm * tj * (3.0f / 2.0f)) / (Am * Jm), ((Am * Am) * (-3.0f / 2.0f) - safe_sqrt((Am * Am * Am * Am) * (1.0f / 4.0f) + (Jm * Jm) * (V0 * V0) + (Am * Am) * (Jm * Jm) * (tj * tj) * (1.0f / 4.0f) + Am * (Jm * Jm) * L * 2.0f - (Am * Am) * Jm * V0 + (Am * Am * Am) * Jm * tj * (1.0f / 2.0f) - Am * (Jm * Jm) * V0 * tj) - Jm * V0 - Am * Jm * tj * (3.0f / 2.0f)) / (Am * Jm)));
            t6_out = t2_out;
        }
    }
    Jm_out = Jm;
}

// generate three consecutive segments forming a jerk profile
// the index variable is the position within the path array that this jerk profile should be added
// the index is incremented to reference the next segment in the array after the jerk profile
void SCurve::add_segments_jerk(uint8_t &index, float tj, float Jm, float Tcj)
{
    add_segment_incr_jerk(index, tj, Jm);
    add_segment_const_jerk(index, Tcj, Jm);
    add_segment_decr_jerk(index, tj, Jm);
}

// generate constant jerk time segment
// calculate the information needed to populate the constant jerk segment from the segment duration tj and jerk J0
// the index variable is the position of this segment in the path array and is incremented to reference the next segment in the array
void SCurve::add_segment_const_jerk(uint8_t &index, float tj, float J0)
{
    const float J = J0;
    const float T = segment[index - 1].end_time + tj;
    const float A = segment[index - 1].end_accel + J0 * tj;
    const float V = segment[index - 1].end_vel + segment[index - 1].end_accel * tj + 0.5f * J0 * sq(tj);
    const float P = segment[index - 1].end_pos + segment[index - 1].end_vel * tj + 0.5f * segment[index - 1].end_accel * sq(tj) + (1.0f / 6.0f) * J0 * powf(tj, 3.0f);
    add_segment(index, T, SegmentType::CONSTANT_JERK, J, A, V, P);
}

// generate increasing jerk magnitude time segment based on a raised cosine profile
// calculate the information needed to populate the increasing jerk magnitude segment from the segment duration tj and jerk magnitude Jm
// the index variable is the position of this segment in the path array and is incremented to reference the next segment in the array
void SCurve::add_segment_incr_jerk(uint8_t &index, float tj, float Jm)
{
    const float Beta = M_PI / tj;
    const float Alpha = Jm / 2.0f;
    const float AT = Alpha * tj;
    const float VT = Alpha * (sq(tj) / 2.0f - 2.0f / sq(Beta));
    const float PT = Alpha * ((-1.0f / sq(Beta)) * tj + (1.0f / 6.0f) * powf(tj, 3.0f));

    const float J = Jm;
    const float T = segment[index - 1].end_time + tj;
    const float A = segment[index - 1].end_accel + AT;
    const float V = segment[index - 1].end_vel + segment[index - 1].end_accel * tj + VT;
    const float P = segment[index - 1].end_pos + segment[index - 1].end_vel * tj + 0.5f * segment[index - 1].end_accel * sq(tj) + PT;
    add_segment(index, T, SegmentType::POSITIVE_JERK, J, A, V, P);
}

// generate decreasing jerk magnitude time segment based on a raised cosine profile
// calculate the information needed to populate the decreasing jerk magnitude segment from the segment duration tj and jerk magnitude Jm
// the index variable is the position of this segment in the path and is incremented to reference the next segment in the array
void SCurve::add_segment_decr_jerk(uint8_t &index, float tj, float Jm)
{
    const float Beta = M_PI / tj;
    const float Alpha = Jm / 2.0f;
    const float AT = Alpha * tj;
    const float VT = Alpha * (sq(tj) / 2.0f - 2.0f / sq(Beta));
    const float PT = Alpha * ((-1.0f / sq(Beta)) * tj + (1.0f / 6.0f) * powf(tj, 3.0f));
    const float A2T = Jm * tj;
    const float V2T = Jm * sq(tj);
    const float P2T = Alpha * ((-1.0f / sq(Beta)) * 2.0f * tj + (4.0f / 3.0f) * powf(tj, 3.0f));

    const float J = Jm;
    const float T = segment[index - 1].end_time + tj;
    const float A = (segment[index - 1].end_accel - AT) + A2T;
    const float V = (segment[index - 1].end_vel - VT) + (segment[index - 1].end_accel - AT) * tj + V2T;
    const float P = (segment[index - 1].end_pos - PT) + (segment[index - 1].end_vel - VT) * tj + 0.5f * (segment[index - 1].end_accel - AT) * sq(tj) + P2T;
    add_segment(index, T, SegmentType::NEGATIVE_JERK, J, A, V, P);
}

// add single S-Curve segment
// populate the information for the segment specified in the path by the index variable.
// the index variable is incremented to reference the next segment in the array
void SCurve::add_segment(uint8_t &index, float end_time, SegmentType seg_type, float jerk_ref, float end_accel, float end_vel, float end_pos)
{
    segment[index].end_time = end_time;
    segment[index].seg_type = seg_type;
    segment[index].jerk_ref = jerk_ref;
    segment[index].end_accel = end_accel;
    segment[index].end_vel = end_vel;
    segment[index].end_pos = end_pos;
    index++;
}

// set speed and acceleration limits for the path
// origin and destination are offsets from EKF origin
// speed and acceleration parameters are given in horizontal, up and down.
void SCurve::set_kinematic_limits(const Vector3f &origin, const Vector3f &destination,
                                  float speed_xy, float speed_up, float speed_down,
                                  float accel_xy, float accel_z)
{
    // ensure arguments are positive
    speed_xy = fabsf(speed_xy);
    speed_up = fabsf(speed_up);
    speed_down = fabsf(speed_down);
    accel_xy = fabsf(accel_xy);
    accel_z = fabsf(accel_z);

    Vector3f direction = destination - origin;
    const float track_speed_max = kinematic_limit(direction, speed_xy, speed_up, speed_down);
    const float track_accel_max = kinematic_limit(direction, accel_xy, accel_z, accel_z);

    vel_max = track_speed_max;
    accel_max = track_accel_max;
}

#if 0
// debugging messages
void SCurve::debug()
{
    hal.console->printf("\n");
    hal.console->printf("num_segs:%u, time:%4.2f, jerk_time:%4.2f, jerk_max:%4.2f, accel_max:%4.2f, vel_max:%4.2f\n",
                        (unsigned)num_segs, (double)time, (double)jerk_time, (double)jerk_max, (double)accel_max, (double)vel_max);
    hal.console->printf("T, Jt, J, A, V, P \n");
    for (uint8_t i = 0; i < num_segs; i++) {
        hal.console->printf("i:%u, T:%4.2f, Jtype:%4.2f, J:%4.2f, A:%4.2f, V: %4.2f, P: %4.2f\n",
                            (unsigned)i, (double)segment[i].end_time, (double)segment[i].seg_type, (double)segment[i].jerk_ref,
                            (double)segment[i].end_accel, (double)segment[i].end_vel, (double)segment[i].end_pos);
    }
    hal.console->printf("track x:%4.2f, y:%4.2f, z:%4.2f\n", (double)track.x, (double)track.y, (double)track.z);
    hal.console->printf("delta_unit x:%4.2f, y:%4.2f, z:%4.2f\n", (double)delta_unit.x, (double)delta_unit.y, (double)delta_unit.z);
    hal.console->printf("\n");
}
#endif
