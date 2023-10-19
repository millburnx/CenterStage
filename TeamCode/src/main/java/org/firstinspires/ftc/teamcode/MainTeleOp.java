package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.*;

@TeleOp(name="Main TeleOp")
public class MainTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        double rollPower = 0.7;

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

//            if(gamepad1.right_bumper) {
//                robot.lift.setMotors(0.6);
//            }
//            else {
//                robot.lift.setMotors(0);
//            }
//
//            if(gamepad1.left_bumper) {
//                robot.lift.setMotors(-0.6);
//            }
//            else {
//                robot.lift.setMotors(0);
//            }

            Pose2d pose = robot.drive.getPos();

            telemetry.addData("Field Centric: ", robot.drive.isFieldCentric);
            telemetry.addData("Power: ", power);
            telemetry.addData("Strafe: ", strafe);
            telemetry.addData("Turn: ", turn);

            telemetry.addData("x: ", pose.getX());
            telemetry.addData("y: ", pose.getY());
            telemetry.addData("heading: ", pose.getHeading());

            telemetry.addData("rightLift: ", robot.lift.rightLift.motorEx.getCurrentPosition());
            telemetry.addData("leftLift: ", robot.lift.leftLift.motorEx.getCurrentPosition());

            telemetry.update();
            robot.dashTelemetry.drawField(pose, dashboard);

            robot.drive.update();
        }
    }
}