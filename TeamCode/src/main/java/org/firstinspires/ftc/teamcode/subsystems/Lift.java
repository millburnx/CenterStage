package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.HardwareDevice;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

public class Lift {
    public MotorEx rightLift;
    public MotorEx leftLift;
    private List<MotorEx> motors;
    public Lift(HardwareMap hardwareMap) {
        rightLift = new MotorEx(hardwareMap, "rightLift", Motor.GoBILDA.RPM_435);
        leftLift = new MotorEx(hardwareMap, "leftLift", Motor.GoBILDA.RPM_435);
        motors = Arrays.asList(rightLift, leftLift);
        for (MotorEx motor : motors) {
            motor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.FLOAT);
            motor.set(0);
            motor.motorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.resetEncoder();
        }
        leftLift.motorEx.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void extendLift(double position) {
//        while(rightLift.motorEx.getCurrentPosition() < position) {
//            rightLift.set(0.2);
//        }
//        rightLift.set(0);
        setMotors(0.6);
    }

    public void setMotors(double power) {
        rightLift.set(power);
        leftLift.set(power);
    }
}
