package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.RR_Robot;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Config
@TeleOp
public class teleopmaybeV2 extends OpMode {
    Robot robot;
    RR_Robot rr_robot;
    Pose2d pose;
    FtcDashboard dashboard;
    TelemetryPacket packet;
    boolean depositToggle;
    double rollPower;

    public DcMotorEx leftLift;
    public DcMotorEx rightLift;

    private PIDController controller;
    public static double p = 0.01, i = 0, d = 0.001;
    public static double f = 0.001;

    public static int target = 0;

    private final double ticks_in_degree = 8192/360.0;
    private int liftIndex = 0;
    private int[] liftHeights = {0,1130,1700,1720};
    boolean left_bumper_pressed = false;
    boolean right_bumper_pressed = false;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, gamepad1);
        rr_robot = new RR_Robot(hardwareMap, gamepad1);
        pose = new com.acmerobotics.roadrunner.geometry.Pose2d(rr_robot.drive.getPose().getX(), rr_robot.drive.getPose().getY(), rr_robot.drive.getPose().getHeading());
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        depositToggle = false;
        rollPower = 1;
        robot.intake.roll(rollPower);
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setDirection(DcMotor.Direction.REVERSE);
        left_bumper_pressed = false;
        right_bumper_pressed = false;
        robot.servoDeposit.rightDeposit.setPosition(0.32);
        robot.servoDeposit.leftDeposit.setPosition(0.32);
        target = 0;

    }

    @Override
    public void loop() {
        boolean togglePressed = false;

        //roller
        if(gamepad1.x) {
            if (!togglePressed) {
                togglePressed = true;
                rollPower = -rollPower;
                if (rollPower < 0) {
                    robot.intake.roll(rollPower );
                } else {
                    robot.intake.roll(rollPower);
                }
            }
        } else {
            togglePressed = false;
        }

        if (gamepad1.a) {
            target = 0;
            liftIndex = 0;
        } else if (gamepad1.b) {
            target = 400;
        }

        if(gamepad1.y){
            robot.drone.setPosition(Math.toRadians(-50));
        }

        //movement

        double power = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        robot.drive.moveTeleOp(power, strafe, turn);

        //lift
        controller.setPID(p, i, d);
        int rightPos = rightLift.getCurrentPosition();
        double pid = controller.calculate(rightPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double liftPower = pid + ff;

        rightLift.setPower(liftPower);
        leftLift.setPower(liftPower);

        if(gamepad1.right_stick_button){
            if (!right_bumper_pressed) {
                if (liftIndex < 3) {
                    liftIndex += 1;
                    target = liftHeights[liftIndex];
                }
            }
            right_bumper_pressed = true;
        } else {
            right_bumper_pressed = false;
        }
        if(gamepad1.left_stick_button){
            if (!left_bumper_pressed) {
                if (liftIndex > 0) {
                    liftIndex -= 1;
                    target = liftHeights[liftIndex];
                }
            }
            left_bumper_pressed = true;
        } else {
            left_bumper_pressed = false;
        }

        //deposit

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

        telemetry.addData("rightPos: ", rightPos);
        telemetry.addData("leftPos: ", leftLift.getCurrentPosition());
        telemetry.addData("rightTarget: ", target);
    }
}
