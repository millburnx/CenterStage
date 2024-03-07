package org.firstinspires.ftc.teamcode.common.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.common.subsystems.Intake;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive rrDrive = new SampleMecanumDrive(hardwareMap);
        MotorEx stageOne = new MotorEx(hardwareMap, "stageOne", Motor.GoBILDA.RPM_1620);
        stageOne.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        stageOne.set(0);
        rrDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            rrDrive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
//            rrDrive.setMotorPowers(0,0,0,0);

            rrDrive.update();

            Pose2d poseEstimate = rrDrive.getPoseEstimate();
            telemetry.addData("Left Pos: ", stageOne.getCurrentPosition());
            telemetry.addData("Right Pos: ", rrDrive.rightRear.getCurrentPosition());
            telemetry.addData("Middle Pos: ", rrDrive.leftRear.getCurrentPosition());

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
