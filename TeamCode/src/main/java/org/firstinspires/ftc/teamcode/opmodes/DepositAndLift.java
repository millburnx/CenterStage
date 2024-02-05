package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.ServoEx;
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
public class DepositAndLift extends OpMode {
    public DcMotorEx leftLift;
    public DcMotorEx rightLift;
    SimpleServo leftDeposit;
    SimpleServo rightDeposit;
    public static double pos1 = 0.5;
    public static double p = 0.01, i = 0, d = 0.001;
    public static double f = 0.001;
    private final double ticks_in_degree = 8192/360.0;

    public static int target = 0;

    public static int position;

    ServoEx blocker;

    TelemetryPacket packet;
    PIDController controller;
    MotorEx stageOne;
    public static int intake;


    @Override
    public void init() {
        stageOne = new MotorEx(hardwareMap, "stageOne", Motor.GoBILDA.RPM_1620);
        stageOne.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        stageOne.set(0);
        //stageOne.motorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        packet = new TelemetryPacket();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setDirection(DcMotor.Direction.REVERSE);

        leftDeposit = new SimpleServo(hardwareMap,"leftDeposit", 0, 360, AngleUnit.DEGREES);
        rightDeposit = new SimpleServo(hardwareMap,"rightDeposit", 0, 360, AngleUnit.DEGREES);

        rightDeposit.setInverted(true);
        rightDeposit.setPosition(0);
        leftDeposit.setPosition(0);
        controller = new PIDController(p, i, d);

        blocker = new SimpleServo(
                hardwareMap, "blocker", 0, 360, AngleUnit.DEGREES
        );
        blocker.setPosition(0);


    }

    @Override
    public void loop() {
        blocker.setPosition(0.4);

        stageOne.set(intake);
        controller.setPID(p, i, d);
        int rightPos = rightLift.getCurrentPosition();
        double pid = controller.calculate(rightPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
        double power = pid + ff;

        rightLift.setPower(power);
        leftLift.setPower(power);


        leftDeposit.setPosition(pos1);
        rightDeposit.setPosition(pos1);
        if(gamepad1.a){
            telemetry.addLine("pos1");
            leftDeposit.setPosition(pos1);
            rightDeposit.setPosition(pos1);

        }
        else if(gamepad1.b){
            telemetry.addLine("0");
            leftDeposit.setPosition(0);
            rightDeposit.setPosition(0);
        }
        telemetry.addData("Left position", leftLift.getCurrentPosition());
        telemetry.addData("Right position", rightLift.getCurrentPosition());


    }
}