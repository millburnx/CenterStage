package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Robot {
    public Drive drive;
    public Intake intake;
    public LiftPID lift;
    public Deposit deposit;
    public Deposit servoDeposit;
    public Odometry odom;
    public PID pid;
    public DashTelemetry dashTelemetry;

    public ServoEx drone;

    public Robot(HardwareMap hardwareMap, Gamepad gamepad) {
        drive = new Drive(hardwareMap);
        intake = new Intake(hardwareMap);
//        lift = new Lift(hardwareMap, gamepad);
//        lift = new LiftPID(hardwareMap);
        servoDeposit = new Deposit(hardwareMap);
        dashTelemetry = new DashTelemetry();
        drone = new SimpleServo(
                hardwareMap, "drone", 0, 120, AngleUnit.DEGREES
        );
    }
}
