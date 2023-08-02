package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.Arrays;
import java.util.List;

public class Drive {
    public boolean isFieldCentric;
    public BNO055IMU imu;
    public MotorEx leftFront;
    public MotorEx leftRear;
    public MotorEx rightFront;
    public MotorEx rightRear;
    private List<MotorEx> motors;

    public Drive(HardwareMap hardwareMap) {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        leftFront = new MotorEx(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_435);
        leftRear = new MotorEx(hardwareMap, "backLeft", Motor.GoBILDA.RPM_435);
        rightFront = new MotorEx(hardwareMap, "frontRight", Motor.GoBILDA.RPM_435);
        rightRear = new MotorEx(hardwareMap, "backRight", Motor.GoBILDA.RPM_435);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (MotorEx motor : motors) {
            motor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
            motor.set(0);
        }
    }

    public void moveTeleOp(double power, double strafe, double turn) {
        if(isFieldCentric) {
            fieldCentric(power, strafe, turn);
        } else {
            robotCentric(power, strafe, turn);
        }
    }

    public void robotCentric(double power, double strafe, double turn) {
        double denominator = Math.max(Math.abs(power) + Math.abs(strafe) + Math.abs(turn), 1);
        double frontLeftPower = (power + strafe + turn) / denominator;
        double backLeftPower = (power - strafe + turn) / denominator;
        double frontRightPower = (power - strafe - turn) / denominator;
        double backRightPower = (power + strafe - turn) / denominator;

        leftFront.set(frontLeftPower);
        leftRear.set(backLeftPower);
        rightFront.set(frontRightPower);
        rightRear.set(backRightPower);
    }

    public void fieldCentric(double power, double strafe, double turn) {
        double botHeading = 0;
        double rotationX = strafe * Math.cos(botHeading) - power * Math.sin(botHeading);
        double rotationY = strafe * Math.sin(botHeading) + power * Math.cos(botHeading);
        double denominator = Math.max(Math.abs(power) + Math.abs(strafe) + Math.abs(turn), 1);
        double frontLeftPower = (rotationY + rotationX + turn) / denominator;
        double backLeftPower = (rotationY - rotationX + turn) / denominator;
        double frontRightPower = (rotationY - rotationX - turn) / denominator;
        double backRightPower = (rotationY + rotationX - turn) / denominator;

        leftFront.set(frontLeftPower);
        leftRear.set(backLeftPower);
        rightFront.set(frontRightPower);
        rightRear.set(backRightPower);
    }

    public void switchDrive() {
        isFieldCentric = !isFieldCentric;
    }
}
