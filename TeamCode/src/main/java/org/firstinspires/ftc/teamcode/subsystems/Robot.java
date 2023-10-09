package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

public class Robot {
    public Drive drive;
    public Odometry odom;
    public PID pid;
    public DashTelemetry dashTelemetry;

    public Robot(HardwareMap hardwareMap) {
        drive = new Drive(hardwareMap);
        odom = new Odometry(drive);
        pid = new PID();
        dashTelemetry = new DashTelemetry();
    }
}
