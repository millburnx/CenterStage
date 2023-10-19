package org.firstinspires.ftc.teamcode.ctrl;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auton.Config;
import org.firstinspires.ftc.teamcode.subsystems.MotionProfile;
import org.firstinspires.ftc.teamcode.subsystems.PID;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

import java.util.function.Supplier;

public class Movement {
    private Drive drive;
    private PID driveXPID;
    private PID driveYPID;
    private PID headingPID;
    private Supplier<Boolean> opModeIsActive;
    private double tolerance;
    private Telemetry telemetry;

    private ElapsedTime timer;

    public Movement(Drive drive, Supplier<Boolean> opModeIsActive, PID.Config drivePIDConfig, PID.Config headingPIDConfig, double tolerance, Telemetry telemetry) {
        this.drive = drive;
        this.driveXPID = new PID(drivePIDConfig);
        this.driveYPID = new PID(drivePIDConfig);
        this.headingPID = new PID(headingPIDConfig);
        this.opModeIsActive = opModeIsActive;
        this.tolerance = tolerance;
        this.telemetry = telemetry;

        timer = new ElapsedTime();
    }

    public void move(Pose2d target) {
        Pose2d init = drive.getPos();
        Pose2d init_target_pose = new Pose2d(target.getX() - init.getX(), target.getY() - init.getY(), Math.toRadians(utils.angleDifference(target.getHeading(), init.getHeading())));
        timer.reset();
        double elapsed_time;
        Pose2d pose = init;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashTelem = dashboard.getTelemetry();

        while (opModeIsActive.get() && (Math.abs(target.getX() - pose.getX()) > tolerance || Math.abs(target.getY() - pose.getY()) > tolerance || Math.abs(utils.angleDifference(target.getHeading(), pose.getHeading())) > 3.0)) {
            elapsed_time = timer.seconds();
            drive.update();
            pose = drive.getPos();
            double instantTargetPositionX = MotionProfile.motion_profile(Config.MAX_ACCEL, Config.MAX_VELOCITY, init_target_pose.getX(), elapsed_time) + init.getX();
            double instantTargetPositionY = MotionProfile.motion_profile(Config.MAX_ACCEL, Config.MAX_VELOCITY, init_target_pose.getY(), elapsed_time) + init.getY();
            double instantTargetPositionH = MotionProfile.motion_profile(Config.MAX_ACCEL, Config.MAX_VELOCITY, init_target_pose.getHeading(), elapsed_time)  + Math.toDegrees(init.getHeading());
            double x = driveXPID.getValue(target.getX() - pose.getX());
            double y = driveYPID.getValue(target.getY() - pose.getY());
            double rx = headingPID.getValue(utils.angleDifference(target.getHeading(), Math.toDegrees(pose.getHeading())));

            double botHeading = Math.toRadians(-pose.getHeading());

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX = rotX * 1.1;
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            telemetry.addData("X motion ", instantTargetPositionX);
            telemetry.addData("Y motion ", instantTargetPositionY);
            telemetry.addData("H motion ", instantTargetPositionH);
            telemetry.addData("X init ", init_target_pose.getX());
            telemetry.addData("Pose ", pose);
            telemetry.addData("XXXX ", x);
            telemetry.addData("YYYY ", y);
            telemetry.addData("x Error ", x);
            telemetry.addData("y Error ", y);
            telemetry.addData("h Error ", rx);
            telemetry.addData("botHeading", pose.getHeading());
            telemetry.addData("rotX  ", rotX);
            telemetry.addData("rotY  ", rotY);
            telemetry.update();

            dashTelem.addData("x", pose.getX());
            dashTelem.addData("y", pose.getY());
            dashTelem.addData("heading", pose.getHeading());
            dashTelem.addData("x target", target.getX());
            dashTelem.addData("y target", target.getY());
            dashTelem.addData("heading target", target.getHeading());
            dashTelem.addData("x Error", x);
            dashTelem.addData("y Error", y);
            dashTelem.addData("rel X", rotX);
            dashTelem.addData("rel Y", rotY);
            dashTelem.addData("heading Error", rx);
            dashTelem.update();

            drive.setDrivePowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        }
        drive.setDrivePowers(0,0,0,0);
    }
}