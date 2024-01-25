package org.firstinspires.ftc.teamcode.subsystems;

public class MotionProfile {
    public static double motion_profile(double max_acceleration, double max_velocity, double distance, double elapsed_time) {

        // calculate the time it takes to accelerate to max velocity
        double acceleration_dt = max_velocity / max_acceleration;

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        double halfway_distance = distance / 2.0;
        double acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2.0);

        if (acceleration_distance > halfway_distance) {
            acceleration_dt = Math.sqrt(Math.abs(halfway_distance) / (0.5 * max_acceleration)) * halfway_distance > 0 ? 1 : -1; // prevent negative sqrt
        }

        acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);

        // recalculate max velocity based on the time we have to accelerate and decelerate
        max_velocity = max_acceleration * acceleration_dt;

        // we decelerate at the same rate as we accelerate
        double deacceleration_dt = acceleration_dt;

        // calculate the time that we're at max velocity
        double cruise_distance = distance - 2 * acceleration_distance;
        double cruise_dt = cruise_distance / max_velocity;
        double deacceleration_time = acceleration_dt + cruise_dt;

        // check if we're still in the motion profile
        double entire_dt = acceleration_dt + cruise_dt + deacceleration_dt;
        if (elapsed_time > entire_dt)
            return distance;

        // if we're accelerating
        if (elapsed_time < acceleration_dt) {
            // use the kinematic equation for acceleration
            return 0.5 * max_acceleration * Math.pow(elapsed_time, 2);
        } else if (elapsed_time < deacceleration_time) { // if we're cruising
            acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);
            double cruise_current_dt = elapsed_time - acceleration_dt;

            // use the kinematic equation for constant velocity
            return acceleration_distance + max_velocity * cruise_current_dt;
        } else { // if we're decelerating
            acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);
            cruise_distance = max_velocity * cruise_dt;
            deacceleration_time = elapsed_time - deacceleration_time;

            // use the kinematic equations to calculate the instantaneous desired position
            return acceleration_distance + cruise_distance + max_velocity * deacceleration_time - 0.5 * max_acceleration * Math.pow(deacceleration_time, 2);
        }
    }
}