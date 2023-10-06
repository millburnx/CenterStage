package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ctrl.MoveTo;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Config
@Autonomous(name="autoTest", group="Linear Opmode")
public class autoTest extends LinearOpMode {
    Pose2d pose;
    Pose2d desiredPose;
    MoveTo moveTo;
    Robot robot;

    public void runOpMode() {
        waitForStart();

        telemetry.addData("Status", "Initialized");

        robot = new Robot(hardwareMap);
        moveTo = new MoveTo(robot, telemetry);

        run(0, 20, 0);
    }

    public void run(double x, double y, double heading) {
        desiredPose = new Pose2d(x, y, heading);
        robot.odom.update();
        pose = robot.odom.getPos();
        Pose2d startPose = pose;
        moveTo.point(pose, desiredPose, startPose, true);
        while(!moveTo.isDone && opModeIsActive()) {
            robot.odom.update();
            pose = robot.odom.getPos();
            moveTo.point(pose, desiredPose, startPose, false);
            telemetry.update();
        }
    }
}
