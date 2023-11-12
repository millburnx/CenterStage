package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.ObjectDetector;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.RR_Robot;

@TeleOp(name="BasicLeftAuton")
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
                .lineToLinearHeading(new Pose2d(30, -1, Math.toRadians(-95)))
                .addTemporalMarker(3, () -> {
                    outtaking = true;
                })
                .build();


        middle_0 = robot.drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(25,4,0))
                .addTemporalMarker(2, () -> {
                    outtaking = true;
                })
                .build();


        right_0 = robot.drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(28, 0, Math.toRadians(95)))
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
            robot.intake.roll(-0.3);
        }
        else{
            robot.intake.roll(0);
        }
    }

    public void depositAsync(){
        if(deposit){
            robot.deposit.rightDeposit.set(1);
            robot.deposit.leftDeposit.set(1);
        }
        else{
            robot.deposit.rightDeposit.set(0);
            robot.deposit.leftDeposit.set(0);
        }
    }
    public void liftAsync(){
        if(up){
            robot.lift.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.lift.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.lift.rightLift.setPower(0.8);
            robot.lift.leftLift.setPower(0.8);

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
