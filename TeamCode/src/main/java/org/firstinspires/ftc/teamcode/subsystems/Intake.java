package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    public MotorEx intakeRoller;
    public Intake(HardwareMap hardwareMap) {
        intakeRoller = new MotorEx(hardwareMap, "intakeRoller", Motor.GoBILDA.RPM_1620);

        intakeRoller.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        intakeRoller.set(0);
        intakeRoller.motorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void roll(double power) {
        intakeRoller.set(power);
    }
}
