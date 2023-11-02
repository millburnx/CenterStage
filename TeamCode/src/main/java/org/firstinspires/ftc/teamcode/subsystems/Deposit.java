package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class Deposit {
    public static final double intakeval = 1;
    public static final double holdval = 0.11;
    public static final double outtakeval = 0;


    public Servo deposit1;
    public Servo deposit2;
    public Deposit(HardwareMap hardwareMap)
    {
        this.deposit1 = hardwareMap.get(Servo.class, "deposit1");
        this.deposit2 = hardwareMap.get(Servo.class, "deposit2");


        deposit1.setPosition(intakeval);
        deposit2.setPosition(intakeval);

    }

    public void intakeDeposit()
    {
        deposit1.setPosition(intakeval);
        deposit2.setPosition(intakeval);

    }

    public void holdDeposit()
    {
        deposit1.setPosition(holdval);
        deposit2.setPosition(holdval);

    }
    public void outtakeDeposit()
    {
        deposit1.setPosition(outtakeval);
        deposit2.setPosition(outtakeval);

    }
}