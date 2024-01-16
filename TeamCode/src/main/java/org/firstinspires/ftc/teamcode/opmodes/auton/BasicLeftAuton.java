package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.opmodes.testing.ObjectDetector;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.RR_Robot;

@Autonomous(name="BasicLeftAuton", group="Autonomous")
public class BasicLeftAuton extends OpMode {
    private int region;
    Trajectory left_0, left_1, left_2, left_3;
    Trajectory middle_0, middle_1, middle_2, middle_3;
    Trajectory right_0, right_1, right_2, right_3;
    boolean activated = false;
    boolean outtaking = false;
    boolean up = false;
    boolean deposit = false;

    ObjectDetector detector;

    RR_Robot robot;

    @Override
    public void init(){
        detector = new ObjectDetector(hardwareMap, telemetry);
    }

    @Override
    public void init_loop(){
        region = detector.getRegion();
        telemetry.addLine(Integer.toString(region));
        telemetry.update();
    }

    @Override
    public void start(){


        robot = new RR_Robot(hardwareMap, gamepad1);

        left_0 = robot.drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(30, 1, Math.toRadians(-95)),
                        SampleMecanumDrive.getVelocityConstraint(18, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(3, () -> {
                    outtaking = true;
                })
                .build();


        middle_0 = robot.drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(26,2,0),
                        SampleMecanumDrive.getVelocityConstraint(18, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(2, () -> {
                    outtaking = true;
                })
                .build();


        right_0 = robot.drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(28, 0, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(18, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(3, () -> {
                    outtaking = true;
                })
                .build();

        // auton goes here
        if  (region == 2) {
            robot.drive.followTrajectoryAsync(right_0);
        } else if (region == 1) {
            robot.drive.followTrajectoryAsync(middle_0);
        } else {
            robot.drive.followTrajectoryAsync(left_0);
        }

        activated = true;
        telemetry.addData("activated: ", activated);
    }

    public void intakeAsync(){
        if(outtaking){
            robot.intake.roll(-0.35);
        }
        else{
            robot.intake.roll(0);
        }
    }

    @Override
    public void loop(){
        if(activated){
            intakeAsync();
            robot.drive.update();
            telemetry.addLine("loop");
            telemetry.update();
        }
    }
}
