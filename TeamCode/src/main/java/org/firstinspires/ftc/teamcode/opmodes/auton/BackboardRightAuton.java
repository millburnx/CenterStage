package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.opmodes.testing.ObjectDetector;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.RR_Robot;

@Autonomous(name="BackboardRightAuton", group="Autonomous")
public class BackboardRightAuton extends OpMode {
    private int region;
    Trajectory left_0, left_1, left_2, left_2_5, left_3;
    Trajectory middle_0, middle_1, middle_2, middle_2_5, middle_3;
    Trajectory right_0, right_1, right_2, right_3, right_2_5, right_4;
    boolean activated = false;
    boolean outtaking = false;
    boolean up = false;
    boolean down = false;
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
                .lineToLinearHeading(new Pose2d(29, 0, Math.toRadians(90)))
                .addTemporalMarker(4, () -> {
                    outtaking = true;
                    robot.drive.followTrajectoryAsync(left_1);
                })
                .build();
        left_1 = robot.drive.trajectoryBuilder(left_0.end())
                .lineToLinearHeading(new Pose2d(29,-33, Math.toRadians(90)))
                .addTemporalMarker(4, ()->{
                    up = true;
                    deposit = true;
                    robot.drive.followTrajectoryAsync(left_2);
                })
                .build();
        left_2 = robot.drive.trajectoryBuilder(left_1.end())
                .lineToLinearHeading(new Pose2d(29, -36, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(2,()->{
                    up = false;
                    deposit = false;
                    robot.drive.followTrajectoryAsync(left_2_5);
                } )
                .build();
        left_2_5 = robot.drive.trajectoryBuilder(left_2.end())
                .lineToLinearHeading(new Pose2d(29, -35, Math.toRadians(90)))
                .addTemporalMarker(2,()->{
                    down = true;
                    robot.drive.followTrajectoryAsync(left_3);
                } )
                .build();
        left_3 = robot.drive.trajectoryBuilder(left_2_5.end())
                .forward(4)
                .addTemporalMarker(4,()->{
                    down = false;
                } )
                .build();

        middle_0 = robot.drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(26,0,0))
                .addTemporalMarker(4, () -> {
                    outtaking = true;
                    robot.drive.followTrajectoryAsync(middle_1);
                })
                .build();
        middle_1 = robot.drive.trajectoryBuilder(middle_0.end())
                .lineToLinearHeading(new Pose2d(28.25, -30 , Math.toRadians(90)))
                .addTemporalMarker(4, ()->{
                    up = true;
                    deposit = true;
                    robot.drive.followTrajectoryAsync(middle_2);
                })
                .build();
        middle_2 = robot.drive.trajectoryBuilder(middle_1.end())
                .lineToLinearHeading(new Pose2d(28.25, -34.5, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(2,()->{
                    up = false;
                    deposit = false;
                    robot.drive.followTrajectoryAsync(middle_2_5);
                } )
                .build();
        middle_2_5 = robot.drive.trajectoryBuilder(middle_2.end())
                .lineToLinearHeading(new Pose2d(28.25, -34.55, Math.toRadians(90 )))
                .addTemporalMarker(2,()->{
                    down = true;
                    robot.drive.followTrajectoryAsync(middle_3);
                } )
                .build();
        middle_3 = robot.drive.trajectoryBuilder(middle_2_5.end())
                .forward(4)
                .addTemporalMarker(3,()->{
                    down = false;
                } )
                .build();


        right_0 = robot.drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(30, -26, Math.toRadians(80)))
                .addTemporalMarker(3, () -> {
                    outtaking = true;
                    robot.drive.followTrajectoryAsync(right_1);
                })
                .build();

        right_1 = robot.drive.trajectoryBuilder(right_0.end())
                .lineToLinearHeading(new Pose2d(18, -33, Math.toRadians(85)))
                .addTemporalMarker(2,()->{
                    up = true;
                    deposit = true;
                    robot.drive.followTrajectoryAsync(right_2);
                } )
                .build();
        right_2 = robot.drive.trajectoryBuilder(right_1.end())
                .lineToLinearHeading(new Pose2d(18, -35.5, Math.toRadians(85)),
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(1,()->{
                    deposit = false;
                    up = false;
                    robot.drive.followTrajectoryAsync(right_2_5);
                } )
                .build();
        right_2_5 = robot.drive.trajectoryBuilder(right_2.end())
                .lineToLinearHeading(new Pose2d(18, -35.55, Math.toRadians(85)),
                        SampleMecanumDrive.getVelocityConstraint(0.5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(1,()->{
                    down = true;
                    robot.drive.followTrajectoryAsync(right_3);
                } )
                .build();
        right_3 = robot.drive.trajectoryBuilder(right_2_5.end())
                .forward(4)
                .addTemporalMarker(2,()->{
                    down = false;
                } )
                .build();

        // auton goes here
        if  (region == 0) {
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

    public void depositAsync(){
        if(deposit){
            robot.deposit.rightDeposit.setPosition(0.9);
            robot.deposit.leftDeposit.setPosition(0.9);
        }
        else{
            robot.deposit.rightDeposit.setPosition(0.32);
            robot.deposit.leftDeposit.setPosition(0.32);
        }
    }
    public void liftAsync(){
        if(up){
            robot.lift.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.lift.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.lift.rightLift.setPower(0.45);
            robot.lift.leftLift.setPower(0.45);

        }
        else if(down){
            robot.lift.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.lift.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.lift.rightLift.setPower(-0.60);
            robot.lift.leftLift.setPower(-0.60);
        }
        else{
            robot.lift.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.lift.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.lift.leftLift.setPower(0);
            robot.lift.rightLift.setPower(0);

        }
    }
    @Override
    public void loop(){
        if(activated){
            intakeAsync();
            liftAsync();
            depositAsync();
            robot.drive.update();
            telemetry.addLine("loop");
            telemetry.update();
        }
    }
}
