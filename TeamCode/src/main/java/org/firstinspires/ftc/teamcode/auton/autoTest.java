package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.ftdi.eeprom.FT_EE_Ctrl;
import org.firstinspires.ftc.teamcode.ctrl.MoveTo;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Config
@Autonomous(name="autoTest", group="Linear Opmode")
public class autoTest extends LinearOpMode {
    Pose2d pose;
    Pose2d desiredPose;
    MoveTo moveTo;
    Robot robot;

    FtcDashboard dashboard;

    public void runOpMode() {
        waitForStart();

        telemetry.addData("Status", "Initialized");
        dashboard = FtcDashboard.getInstance();

        robot = new Robot(hardwareMap);
        moveTo = new MoveTo(robot, telemetry);

        run(20, 0, 0);
    }

    public void run(double x, double y, double heading) {
        desiredPose = new Pose2d(x, y, heading);
        robot.odom.update();
//        pose = robot.odom.getPos();
        pose = new Pose2d(0, 0, 0);
        Pose2d startPose = pose;
        robot.dashTelemetry.drawFieldAuto(pose, desiredPose, startPose, dashboard);
        moveTo.point(pose, desiredPose, startPose, true);
        while(!moveTo.isDone && opModeIsActive()) {
            robot.odom.update();
            pose = robot.odom.getPos();
            moveTo.point(pose, desiredPose, startPose, false);
            telemetry.update();
            robot.dashTelemetry.drawFieldAuto(pose, desiredPose, startPose, dashboard);
        }
    }
}
