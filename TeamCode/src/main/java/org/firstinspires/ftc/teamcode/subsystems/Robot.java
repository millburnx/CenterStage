package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

public class Robot {
    public Drive drive;
    public Intake intake;
    public Lift2 lift;
    public Deposit deposit;
    public Odometry odom;
    public PID pid;
    public DashTelemetry dashTelemetry;

    public Robot(HardwareMap hardwareMap, Gamepad gamepad) {
        drive = new Drive(hardwareMap);
        intake = new Intake(hardwareMap);
        lift = new Lift2(hardwareMap, gamepad);
        //deposit = new Deposit((hardwareMap));
//        odom = new Odometry(drive);
        dashTelemetry = new DashTelemetry();
    }
}
