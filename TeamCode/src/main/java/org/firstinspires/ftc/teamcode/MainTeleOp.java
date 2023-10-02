package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;

import org.firstinspires.ftc.teamcode.subsystems.*;

@TeleOp(name="TeleOp2")
public class MainTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Odometry odom = new Odometry(robot.drive);

        waitForStart();

        while (opModeIsActive()) {
            double power = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            Pose2d pose = odom.getPos();

            robot.drive.moveTeleOp(power, strafe, turn);

            telemetry.addData("Field Centric: ", robot.drive.isFieldCentric);
            telemetry.addData("Odom: ", robot.odom.odometry.getPose());
            telemetry.addData("Power: ", power);
            telemetry.addData("Strafe: ", strafe);
            telemetry.addData("Turn: ", turn);

            telemetry.addData("x: ", pose.getX());
            telemetry.addData("y: ", pose.getY());
            telemetry.addData("heading: ", pose.getHeading());

            telemetry.update();

            odom.odometry.updatePose();
        }
    }
}