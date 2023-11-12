package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ObjectDetector;
import org.firstinspires.ftc.teamcode.subsystems.RR_Robot;

@TeleOp(name="LiftRightAuton")
public class LiftRightAuton extends LinearOpMode {
    private int region;
    Trajectory left_0, left_1;
    Trajectory middle_0, middle_1;
    Trajectory right_0, right_1;

    @Override
    public void runOpMode(){
        ObjectDetector detector = new ObjectDetector(hardwareMap, telemetry);

        RR_Robot robot = new RR_Robot(hardwareMap, gamepad1);

        left_0 = robot.drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(30, 3, Math.toRadians(-90)))
                .addTemporalMarker(3, () -> {
                    robot.intake.roll(-0.2);
                    robot.drive.followTrajectory(left_1);
                })
                .build();
        left_1 = robot.drive.trajectoryBuilder(left_0.end())
                .back(33)
                .build();

        middle_0 = robot.drive.trajectoryBuilder(new Pose2d())
                .forward(30)
                .addTemporalMarker(3, () -> {
                    robot.intake.roll(-0.2);
                    robot.drive.followTrajectory(middle_1);
                })
                .build();
        middle_1 = robot.drive.trajectoryBuilder(middle_0.end())
                .strafeRight(30)
                .build();

        right_0 = robot.drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(30, -3, Math.toRadians(90)))
                .addTemporalMarker(3, () -> {
                    robot.intake.roll(-0.2);
                    robot.drive.followTrajectory(right_1);
                })
                .build();
        right_1 = robot.drive.trajectoryBuilder(right_0.end())
                .forward(27)
                .build();

        while (!opModeIsActive()) {
            region = detector.getRegion();
            telemetry.addLine(Integer.toString(region));
            telemetry.update();
        }

        // auton goes here
        telemetry.addLine("auton goes here");
        telemetry.update();

        if (isStopRequested()) {
            return;
        }

        if  (region == 0) {
            robot.drive.followTrajectory(left_0);
        } else if (region == 1) {
            robot.drive.followTrajectory(middle_0);
        } else {
            robot.drive.followTrajectory(right_0);
        }
    }
}
