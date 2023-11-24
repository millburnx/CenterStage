package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class RR_Robot {
    public SampleMecanumDrive drive;
    public Intake intake;
    public Lift lift;
    public Deposit deposit;
    public Odometry odom;
    public PID pid;
    public DashTelemetry dashTelemetry;

    public RR_Robot(HardwareMap hardwareMap, Gamepad gamepad) {
        drive = new SampleMecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap);
        lift = new Lift(hardwareMap, gamepad);
        deposit = new Deposit(hardwareMap);
        dashTelemetry = new DashTelemetry();
    }
}
