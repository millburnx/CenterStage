package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;

import org.firstinspires.ftc.teamcode.subsystems.*;

@TeleOp(name="TeleOp")
public class MainTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            double power = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            robot.drive.moveTeleOp(power, strafe, turn);

            telemetry.addData("Field Centric: ", robot.drive.isFieldCentric);
            telemetry.addData("Odom: ", robot.odom.odometry.getPose());
            telemetry.addData("Power: ", power);
            telemetry.addData("Strafe: ", strafe);
            telemetry.addData("Turn: ", turn);

            telemetry.addData("rightFront Pos: ", robot.drive.rightFront.encoder.getPosition());
            telemetry.addData("leftFront Pos: ", robot.drive.leftFront.encoder.getPosition());
            telemetry.addData("rightRear Pos: ", robot.drive.rightRear.encoder.getPosition());
            telemetry.addData("leftRear Pos: ", robot.drive.leftRear.encoder.getPosition());

            telemetry.addData("x: ", robot.drive.x);
            telemetry.addData("y: ", robot.drive.y);
            telemetry.addData("heading: ", robot.drive.heading);

            telemetry.update();

            robot.odom.odometry.updatePose();
        }
    }
}