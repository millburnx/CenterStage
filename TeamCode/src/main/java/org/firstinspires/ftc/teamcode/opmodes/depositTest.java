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
public class depositTest extends OpMode {
    SimpleServo leftDeposit;
    SimpleServo rightDeposit;
    MotorEx stageOne;
    public static double pos1 = 0.5;

    TelemetryPacket packet;


    @Override
    public void init() {
        packet = new TelemetryPacket();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        stageOne = new MotorEx(hardwareMap, "stageOne", Motor.GoBILDA.RPM_1620);
        stageOne.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        stageOne.set(1);
        stageOne.motorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDeposit = new SimpleServo(hardwareMap,"leftDeposit", 0, 180, AngleUnit.DEGREES);
        rightDeposit = new SimpleServo(hardwareMap,"rightDeposit", 0, 180, AngleUnit.DEGREES);

        rightDeposit.setInverted(true);
        rightDeposit.setPosition(0);
        leftDeposit.setPosition(0);


    }

    @Override
    public void loop() {
        leftDeposit.setPosition(pos1);
        rightDeposit.setPosition(pos1);
        if (gamepad1.a) {
            telemetry.addLine("pos1");
            leftDeposit.setPosition(pos1);
            rightDeposit.setPosition(pos1);

        }
        else if (gamepad1.b) {
            telemetry.addLine("0");
            leftDeposit.setPosition(0);
            rightDeposit.setPosition(0);
        }
        telemetry.addData("Left position", leftDeposit.getPosition());
        telemetry.addData("Right position", rightDeposit.getPosition());


    }
}