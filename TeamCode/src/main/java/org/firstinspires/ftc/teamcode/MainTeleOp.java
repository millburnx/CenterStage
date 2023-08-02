package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;

import org.firstinspires.ftc.teamcode.subsystems.*;

@TeleOp(name="TeleOp")
public class MainTeleOp extends LinearOpMode {
    public static final double TRACKWIDTH = 14.7;
    public static final double CENTER_WHEEL_OFFSET = -2.1; // distance between center of rotation of the robot and the center odometer
    public static final double WHEEL_DIAMETER = 2.0;
    public static final double TICKS_PER_REV = 8192;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    private Encoder leftOdom, rightOdom, centerOdom;
    private HolonomicOdometry odometry;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, gamepad1);

        leftOdom = robot.drive.leftFront.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightOdom = robot.drive.rightFront.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        centerOdom = robot.drive.leftRear.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        rightOdom.setDirection(Motor.Direction.REVERSE);

        odometry = new HolonomicOdometry(
                leftOdom::getDistance,
                rightOdom::getDistance,
                centerOdom::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );

        waitForStart();

        while (opModeIsActive()) {
            double power = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x * 1.1;
            double turn = gamepad1.right_stick_x;

            robot.drive.moveTeleOp(power, strafe, turn);

            telemetry.addData("Field Centric: ", robot.drive.isFieldCentric);
            telemetry.addData("Odom: ", odometry.getPose());

            telemetry.update();

            odometry.updatePose();
        }
    }
}