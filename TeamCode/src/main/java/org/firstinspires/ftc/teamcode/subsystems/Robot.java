package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public Drive drive;
    public Intake intake;
    public Lift lift;
    public Deposit deposit;
    public Deposit servoDeposit;
    public Odometry odom;
    public PID pid;
    public DashTelemetry dashTelemetry;

    public Robot(HardwareMap hardwareMap, Gamepad gamepad) {
        drive = new Drive(hardwareMap);
        intake = new Intake(hardwareMap);
        lift = new Lift(hardwareMap, gamepad);
//        deposit = new Deposit((hardwareMap));
        servoDeposit = new Deposit(hardwareMap);
//        odom = new Odometry(drive);
        dashTelemetry = new DashTelemetry();
    }
}
