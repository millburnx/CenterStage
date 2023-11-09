package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ObjectDetector;
import org.firstinspires.ftc.teamcode.subsystems.RR_Robot;

@TeleOp(name="BasicAuton")
public class BasicAuton extends LinearOpMode {
    private int region;
    Trajectory right0, right1;
    @Override
    public void runOpMode(){
        ObjectDetector detector = new ObjectDetector(hardwareMap, telemetry);

        RR_Robot robot = new RR_Robot(hardwareMap, gamepad1);
        right0 = robot.drive.trajectoryBuilder(new Pose2d())
                .forward(24)
                .addTemporalMarker(3, ()->{
                    robot.drive.followTrajectory(right1);
                })
                .build();
        right1 = robot.drive.trajectoryBuilder(right0.end())
                .lineToLinearHeading(new Pose2d(right0.end().getX()+0.01, right0.end().getY(), Math.toRadians(90)))
                .addTemporalMarker(3, ()->{
                    robot.intake.roll(-0.2);
                })
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

        robot.drive.followTrajectory(right0);
    }
}
