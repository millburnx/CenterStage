package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp
public class ServoTest extends OpMode {
    SimpleServo intakeLeft;
    MotorEx stageOne;

    SimpleServo intakeRight;
    SimpleServo rightDeposit;
    SimpleServo leftDeposit;


    public static double pos;
    public static double pos2;

    TelemetryPacket packet;


    @Override
    public void init() {
        pos = 0.15;
        packet = new TelemetryPacket();
        rightDeposit = new SimpleServo(
                hardwareMap, "rightDeposit", 0, 120, AngleUnit.DEGREES
        );
        leftDeposit = new SimpleServo(
                hardwareMap, "leftDeposit", 0, 120, AngleUnit.DEGREES
        );
        rightDeposit.setInverted(true);
        rightDeposit.setPosition(0);
        leftDeposit.setPosition(0);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        stageOne = new MotorEx(hardwareMap, "stageOne", Motor.GoBILDA.RPM_1620);
        stageOne.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        stageOne.set(1);
        stageOne.motorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeLeft = new SimpleServo(
                hardwareMap, "intakeLeft", 0, 180, AngleUnit.DEGREES
        );
        intakeRight = new SimpleServo(
                hardwareMap, "intakeRight", 0, 180, AngleUnit.DEGREES
        );
        intakeLeft.setInverted(true);
        intakeLeft.setPosition(0.15);
        intakeRight.setPosition(0.15);


    }

    @Override

    public void loop() {
        rightDeposit.setPosition(pos2);
        leftDeposit.setPosition(pos2);
        intakeLeft.setPosition(pos);
        intakeRight.setPosition(pos);
    }
}