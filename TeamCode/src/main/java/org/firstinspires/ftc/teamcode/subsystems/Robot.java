package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public Drive drive;
    public Intake intake;
    public LiftPID lift;
    public Deposit deposit;
    public Deposit servoDeposit;
    public Odometry odom;
    public PID pid;
    public DashTelemetry dashTelemetry;

    public Robot(HardwareMap hardwareMap, Gamepad gamepad) {
        drive = new Drive(hardwareMap);
        intake = new Intake(hardwareMap);
//        lift = new Lift(hardwareMap, gamepad);
        lift = new LiftPID(hardwareMap);
        servoDeposit = new Deposit(hardwareMap);
        dashTelemetry = new DashTelemetry();
    }
}
