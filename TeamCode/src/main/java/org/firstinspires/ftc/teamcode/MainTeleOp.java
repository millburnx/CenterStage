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
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        boolean depositToggle = false;

        double rollPower = 0.8;

        waitForStart();

        robot.intake.roll(rollPower);

        while (opModeIsActive()) {
            double power = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            robot.drive.moveTeleOp(power, strafe, turn);

            if(gamepad1.x) {
                rollPower = -rollPower;
                robot.intake.roll(rollPower);
            }

            if(gamepad1.right_trigger>0.5){
                depositToggle = false;
            }

            robot.lift.liftTeleOp(gamepad1);

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

            Pose2d pose = robot.drive.getPos();

            telemetry.addData("deposit toggle", depositToggle);

            telemetry.addData("Field Centric: ", robot.drive.isFieldCentric);
            telemetry.addData("Power: ", power);
            telemetry.addData("Strafe: ", strafe);
            telemetry.addData("Turn: ", turn);

            telemetry.addData("x: ", pose.getX());
            telemetry.addData("y: ", pose.getY());
            telemetry.addData("heading: ", pose.getHeading());


            telemetry.addData("rightLift: ", robot.lift.rightLift.getCurrentPosition());
            telemetry.addData("leftLift: ", robot.lift.leftLift.getCurrentPosition());

//            telemetry.addData("intakeTime: ", robot.deposit.time.milliseconds()-robot.deposit.time.startTime());
            telemetry.addData("leftDeposit: ", robot.servoDeposit.leftDeposit.getPosition());
            telemetry.addData("rightDeposit: ", robot.servoDeposit.rightDeposit.getPosition());

            telemetry.update();
            robot.dashTelemetry.drawField(pose, dashboard);

            robot.drive.update();
        }
    }
}