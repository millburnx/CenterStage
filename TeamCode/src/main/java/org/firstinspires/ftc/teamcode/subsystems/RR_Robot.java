package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

public class RR_Robot {
    public SampleMecanumDrive drive;
    public Intake intake;
    public Lift2 lift;
    public Deposit deposit;
    public Odometry odom;
    public PID pid;
    public DashTelemetry dashTelemetry;

    public RR_Robot(HardwareMap hardwareMap, Gamepad gamepad) {
        drive = new SampleMecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap);
        lift = new Lift2(hardwareMap, gamepad);
        deposit = new Deposit((hardwareMap));
        dashTelemetry = new DashTelemetry();
    }
}
