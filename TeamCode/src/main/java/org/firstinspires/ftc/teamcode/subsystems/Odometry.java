package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Odometry {
    public static final double TRACKWIDTH = 14.7;
    public static final double CENTER_WHEEL_OFFSET = -2.1; // distance between center of rotation of the robot and the center odometer
    public static final double WHEEL_DIAMETER = 2.0;
    public static final double TICKS_PER_REV = 8192;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    private Motor.Encoder leftOdom, rightOdom, centerOdom;
    public HolonomicOdometry odometry;

    public Odometry(Drive drive){
        leftOdom = drive.leftFront.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightOdom = drive.rightRear.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        centerOdom = drive.leftRear.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        rightOdom.setDirection(Motor.Direction.REVERSE);

        odometry = new HolonomicOdometry(
                leftOdom::getDistance,
                rightOdom::getDistance,
                centerOdom::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );
    }

    public void update() {
        odometry.updatePose();
    }

    public Pose2d getPos() {
        return new Pose2d(odometry.getPose().getX(), odometry.getPose().getY(), odometry.getPose().getHeading());
    }
}
