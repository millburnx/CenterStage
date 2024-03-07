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
    SimpleServo servo1;
    SimpleServo servo2;

    public static double pos;
    TelemetryPacket packet;


    @Override
    public void init() {
        pos = 0;
        packet = new TelemetryPacket();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        servo1 = new SimpleServo(
                hardwareMap, "depositHook", 0, 180, AngleUnit.DEGREES
        );

//        servo2 = new SimpleServo(
//                hardwareMap, "intakeRight", 0, 180, AngleUnit.DEGREES
//        );
//        servo1.setInverted(true);

    }

    @Override

    public void loop() {
        servo1.setPosition(pos);

    }
}