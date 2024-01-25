package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.RR_Robot;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Config
@TeleOp
public class LiftPosition extends OpMode {
    Robot robot;
    RR_Robot rr_robot;
    Pose2d pose;
    FtcDashboard dashboard;
    TelemetryPacket packet;
    boolean depositToggle;
    double rollPower;

    public DcMotorEx leftLift;
    public DcMotorEx rightLift;

    private PIDController controller;
    public static double p = 0.01, i = 0, d = 0.001;
    public static double f = 0.001;

    public static int target = 0;

    private final double ticks_in_degree = 8192/360.0;
    private int liftIndex = 0;
    private int[] liftHeights = {0,200,400,600};
    boolean left_bumper_pressed = false;
    boolean right_bumper_pressed = false;

    @Override
    public void init() {
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop() {
        telemetry.addData("rightPos: ", rightLift.getCurrentPosition());
        telemetry.addData("leftPos: ", leftLift.getCurrentPosition());
        telemetry.addData("rightTarget: ", target);
    }
}
