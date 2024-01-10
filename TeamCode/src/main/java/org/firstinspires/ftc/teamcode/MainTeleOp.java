package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.*;

@TeleOp(name="Main TeleOp")
public class MainTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, gamepad1);
        boolean hangSeq1 = false;
        boolean hangSeq2 = false;
        RR_Robot rr_robot = new RR_Robot(hardwareMap, gamepad1);
        Pose2d pose = new Pose2d(rr_robot.drive.getPose().getX(), rr_robot.drive.getPose().getY(), rr_robot.drive.getPose().getHeading());
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        boolean depositToggle = false;

        double rollPower = 0.6;

        robot.lift.setTarget(300);

        waitForStart();

        robot.intake.roll(rollPower);

        boolean togglePressed = false;

        while (opModeIsActive()) {
            double power = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            robot.drive.moveTeleOp(power, strafe, turn);

            if(gamepad1.x) {
                if (!togglePressed) {
                    togglePressed = true;
                    rollPower = -rollPower;
                    if (rollPower < 0) {
                        robot.intake.roll(rollPower + 0.4);
                    } else {
                        robot.intake.roll(rollPower);
                    }
                }
            } else {
                togglePressed = false;
            }

            if(gamepad1.right_trigger>0.5){
                depositToggle = false;
            }

//            robot.lift.liftTeleOp(gamepad1);

            robot.lift.run();

            if(gamepad1.circle) {
                robot.lift.setTarget(1000);
                robot.servoDeposit.rightDeposit.setPosition(0.9);
                robot.servoDeposit.leftDeposit.setPosition(0.9);
            }

            if(gamepad1.triangle) {
                robot.lift.setTarget(0);
                robot.servoDeposit.rightDeposit.setPosition(0.32);
                robot.servoDeposit.leftDeposit.setPosition(0.32);
            }
            if(gamepad1.right_bumper){
//                if(!hangSeq1){
//                    hangSeq1 = true;
//                    robot.lift.setTarget(1000);
//                }
//                else if(hangSeq1){
//                    hangSeq1 = false;
//                    robot.lift.setTarget(200);
//                }
                robot.lift.setTarget(1000);
            }

            if(gamepad1.left_bumper){
                depositToggle = true;
            }

            if (gamepad1.dpad_down) {
                robot.servoDeposit.rightDeposit.setPosition(0.32);
                robot.servoDeposit.leftDeposit.setPosition(0.32);
            } else if (gamepad1.dpad_up) {
                robot.servoDeposit.rightDeposit.setPosition(0.9);
                robot.servoDeposit.leftDeposit.setPosition(0.9);
            } else if (gamepad1.dpad_right) {
                robot.servoDeposit.rightDeposit.setPosition(0.45);
                robot.servoDeposit.leftDeposit.setPosition(0.45);
            }

            pose = new Pose2d(rr_robot.drive.getPoseEstimate().getX(), rr_robot.drive.getPoseEstimate().getY(), rr_robot.drive.getPoseEstimate().getHeading());

            telemetry.addData("deposit toggle", depositToggle);

            telemetry.addData("Field Centric: ", robot.drive.isFieldCentric);
            telemetry.addData("Power: ", power);
            telemetry.addData("Strafe: ", strafe);
            telemetry.addData("Turn: ", turn);

            telemetry.addData("liftTarget: ", robot.lift.target);
            telemetry.addData("rightLift: ", robot.lift.rightLift.getCurrentPosition());
            telemetry.addData("leftLift: ", robot.lift.leftLift.getCurrentPosition());


            telemetry.addData("leftDeposit: ", robot.servoDeposit.leftDeposit.getPosition());
            telemetry.addData("rightDeposit: ", robot.servoDeposit.rightDeposit.getPosition());

            telemetry.update();
            robot.dashTelemetry.drawField(pose, dashboard);

            robot.drive.update();
            rr_robot.drive.update();
        }
    }
}