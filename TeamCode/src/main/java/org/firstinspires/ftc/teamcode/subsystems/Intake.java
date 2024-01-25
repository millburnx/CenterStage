package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Intake {
    public MotorEx intakeRoller;
    public MotorEx intakeUp;
    public ServoEx intakeLeft;
    public ServoEx intakeRight;
    public Intake(HardwareMap hardwareMap) {
        intakeRoller = new MotorEx(hardwareMap, "intakeRoller", Motor.GoBILDA.RPM_1620);
        intakeRoller.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        intakeRoller.set(0);
        intakeRoller.motorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeUp = new MotorEx(hardwareMap, "intakeUp", Motor.GoBILDA.RPM_435);
        intakeUp.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        intakeUp.set(0);
        intakeUp.motorEx.setMode((DcMotor.RunMode.RUN_WITHOUT_ENCODER));

        intakeLeft = new SimpleServo(
                hardwareMap, "intakeLeft", 0, 120, AngleUnit.DEGREES
        );
        intakeRight = new SimpleServo(
                hardwareMap, "intakeRight", 0, 120, AngleUnit.DEGREES
        );
        intakeLeft.setInverted(true);
        intakeLeft.setPosition(0);
        intakeRight.setPosition(0);
    }

    public void roll(double power) {
        intakeRoller.set(power);
        intakeUp.set(-0.6);
    }

    public void rotate(double position) {
        intakeLeft.setPosition(position);
        intakeRight.setPosition(position);
    }
}
