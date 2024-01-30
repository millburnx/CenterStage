package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp
public class intakeLiftTest extends OpMode {
    SimpleServo leftServo;
    SimpleServo rightServo;
    TelemetryPacket packet;


    @Override
    public void init() {
        packet = new TelemetryPacket();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftServo = new SimpleServo(hardwareMap,"intakeLeft", 00, 120, AngleUnit.DEGREES);
        rightServo = new SimpleServo(hardwareMap,"intakeRight", 0, 120, AngleUnit.DEGREES);
        leftServo.setInverted(true);
        leftServo.setPosition(0);
        rightServo.setPosition(0);

    }

    @Override
    public void loop() {
        if(gamepad1.triangle){
            telemetry.addLine("0.5");
            leftServo.setPosition(0.15);
            rightServo.setPosition(0.15);

        }
        else if(gamepad1.circle){
            telemetry.addLine("0");
            leftServo.setPosition(0);
            rightServo.setPosition(0);
        }
        else if(gamepad1.square){
            leftServo.turnToAngle(90);
            rightServo.turnToAngle(90);

        }
// Hi saaz

        telemetry.addData("Left Angle: ", leftServo.getAngle());
        telemetry.addData("Right Angle: ", leftServo.getAngle());
        telemetry.addData("Left Pos: ", leftServo.getPosition());
        telemetry.addData("Right Pos: ", rightServo.getPosition());


    }
}