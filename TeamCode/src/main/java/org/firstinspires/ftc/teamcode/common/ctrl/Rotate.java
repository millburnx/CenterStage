package org.firstinspires.ftc.teamcode.common.ctrl;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.PID;

import org.firstinspires.ftc.teamcode.subsystems.Drive;

public class Rotate {
    private Drive drive;
    private Telemetry telemetry;
    private Supplier<Boolean> opModeIsActive;
    private PID rotatePID;
    private double tolerance;

    public Rotate(Drive drive, Telemetry telemetry, Supplier<Boolean> opModeIsActive, PID rotatePID, double tolerance) {
        this.drive = drive;
        this.telemetry = telemetry;
        this.opModeIsActive = opModeIsActive;
        this.rotatePID = rotatePID;
        this.tolerance = tolerance;
    }

    public void rotate(double targetHeading) {
        drive.update();
        rotatePID.reset();
        double error;
        drive.update();
        while (Math.abs(MathUtils.angleDifference(targetHeading, drive.getPos().getHeading())) > tolerance && opModeIsActive.get()) {
            drive.update();
            error = MathUtils.angleDifference(targetHeading, drive.getPos().getHeading());
            double headingPower = rotatePID.getValue(error);
            drive.moveTeleOp(0, 0, headingPower);
            telemetry.addData("heading ", drive.getPos().getHeading());
            telemetry.addData("error ", error);
            telemetry.addData("headingPower ", headingPower);
            telemetry.update();
        }
    }
}