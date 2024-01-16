package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.LiftPID;

@Config
@TeleOp
public class LiftPIDTuner_2 extends OpMode {
    public LiftPID lift;
    public static double p = 0.01, i = 0, d = 0.001;
    public static double f = 0.001;

    public static int target = 0;

    private final double ticks_in_degree = 8192/360.0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        lift = new LiftPID(hardwareMap);
    }

    @Override
    public void loop() {
        lift.setTarget(target);
        lift.run();

        telemetry.addData("rightPos: ", lift.rightLift.getCurrentPosition());
        telemetry.addData("leftPos: ", lift.leftLift.getCurrentPosition());
        telemetry.addData("rightTarget: ", target);
    }
}
