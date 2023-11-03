package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Deposit {
    public static final double intakeval = 0;
    public static final double holdval = 0.25;
    public static final double outtakeval = 0.7;

    public CRServo rightDeposit;
    public CRServo leftDeposit;
    public ElapsedTime time;
    public Deposit(HardwareMap hardwareMap)
    {
        rightDeposit = new CRServo(hardwareMap, "rightDeposit");
        leftDeposit = new CRServo(hardwareMap, "leftDeposit");
        leftDeposit.setInverted(true);
        rightDeposit.set(0);
        leftDeposit.set(0);
        time = new ElapsedTime();
    }

    public void intakeDeposit()
    {
    }

    public void holdDeposit()
    {
    }
    public void outtakeDeposit()
    {
    }
}