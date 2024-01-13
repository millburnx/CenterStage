package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.ObjectDetector;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.RR_Robot;

@Autonomous(name="BackboardRightAutonNew", group="Autonomous")
public class BackBoardRightAutonNew extends OpMode {
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
    private PIDController controller;

    public static double p = 0.01, i = 0, d = 0.001;
    public static double f = 0.001;

    public static int target = 0;
    private final double ticks_in_degree = 8192/360.0;

    @Override
    public void init(){
        detector = new ObjectDetector(hardwareMap, telemetry);
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
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
                .lineToLinearHeading(new Pose2d(30, 1, Math.toRadians(95)))
                .addTemporalMarker(3, () -> {
                    outtaking = true;
                    robot.drive.followTrajectoryAsync(left_1);
                })
                .build();
        left_1 = robot.drive.trajectoryBuilder(left_0.end())
                .lineToLinearHeading(new Pose2d(35,-33, Math.toRadians(95)))
                .addTemporalMarker(2, ()->{
                    target = 1000;
                    deposit = true;
                    robot.drive.followTrajectoryAsync(left_2);
                })
                .build();
        left_2 = robot.drive.trajectoryBuilder(left_1.end())
                .lineToLinearHeading(new Pose2d(31, -35.5, Math.toRadians(95)),
                        SampleMecanumDrive.getVelocityConstraint(7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(1,()->{
                    target = 0;
                    robot.drive.followTrajectoryAsync(left_3);
                } )
                .build();
        left_3 = robot.drive.trajectoryBuilder(left_2.end())
                .forward(4)
                .addTemporalMarker(2,()->{
                    deposit = false;
                } )
                .build();

        middle_0 = robot.drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(24,-4,0))
                .addTemporalMarker(3, () -> {
                    outtaking = true;
                    robot.drive.followTrajectoryAsync(middle_1);
                })
                .build();
        middle_1 = robot.drive.trajectoryBuilder(middle_0.end())
                .lineToLinearHeading(new Pose2d(27.25, -33 , Math.toRadians(95)))
                .addTemporalMarker(3, ()->{
                    target = 1000;
                    deposit = true;
                    robot.drive.followTrajectoryAsync(middle_2);
                })
                .build();
        middle_2 = robot.drive.trajectoryBuilder(middle_1.end())
                .lineToLinearHeading(new Pose2d(27.25, -36.5, Math.toRadians(95)),
                        SampleMecanumDrive.getVelocityConstraint(7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(1,()->{
                    target = 0;
                    robot.drive.followTrajectoryAsync(middle_3);
                } )
                .build();
        middle_3 = robot.drive.trajectoryBuilder(middle_2.end())
                .forward(4)
                .addTemporalMarker(2,()->{
                    deposit = false;
                } )
                .build();


        right_0 = robot.drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(30, -22, Math.toRadians(90)))
                .addTemporalMarker(3, () -> {
                    outtaking = true;
                    robot.drive.followTrajectoryAsync(right_1);
                })
                .build();

        right_1 = robot.drive.trajectoryBuilder(right_0.end())
                .lineToLinearHeading(new Pose2d(22, -33, Math.toRadians(90)))
                .addTemporalMarker(2,()->{
                    target = 1000;
                    deposit = true;
                    robot.drive.followTrajectoryAsync(right_2);
                } )
                .build();
        right_2 = robot.drive.trajectoryBuilder(right_1.end())
                .lineToLinearHeading(new Pose2d(22, -35.5, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(1,()->{
                    target = 0;
                    robot.drive.followTrajectoryAsync(right_3);
                } )
                .build();
        right_3 = robot.drive.trajectoryBuilder(right_1.end())
                .forward(4)
                .addTemporalMarker(2,()->{
                    deposit = false;
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
            robot.intake.roll(-0.3);
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
        controller.setPID(p, i, d);
        int rightPos = robot.lift.rightLift.getCurrentPosition();
        double pid = controller.calculate(rightPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double liftPower = pid + ff;

        robot.lift.rightLift.setPower(liftPower);
        robot.lift.leftLift.setPower(liftPower);
    }
    @Override
    public void loop(){
        if(activated){
            intakeAsync();
            liftAsync();
            depositAsync();
            robot.drive.update();
            telemetry.addLine("loop");
            telemetry.addData("target", target);
            telemetry.addData("rightLift", robot.lift.rightLift.getCurrentPosition());
            telemetry.update();
        }
    }
}
