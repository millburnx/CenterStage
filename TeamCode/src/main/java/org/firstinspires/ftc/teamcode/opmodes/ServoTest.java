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
public class ServoTest extends OpMode {
    SimpleServo servo;
    public static double pos;
    TelemetryPacket packet;


    @Override
    public void init() {
        pos = 0;
        packet = new TelemetryPacket();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        servo = new SimpleServo(hardwareMap,"blocker", 0, 360, AngleUnit.DEGREES);
        servo.setPosition(100);
    }

    @Override

    public void loop() {
        servo.setPosition(pos);
        if(gamepad1.a){
            telemetry.addLine("90");
            servo.setPosition(0.5);
        }
        else if(gamepad1.b){
            telemetry.addLine("0");
            servo.setPosition(0);
        }
// Hi saaz
        telemetry.addData("Angle: ", servo.getAngle());
        telemetry.addData("Pos: ", servo.getPosition());
        telemetry.addLine("hi");

    }
}