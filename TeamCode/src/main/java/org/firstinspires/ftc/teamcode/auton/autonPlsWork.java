package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ctrl.Movement;
import org.firstinspires.ftc.teamcode.ctrl.Rotate;
import org.firstinspires.ftc.teamcode.subsystems.PID;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous
public class autonPlsWork extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        PID.Config translationConfig = new PID.Config(Config.translationP, Config.translationI, Config.translationD);
        PID.Config rotationConfig = new PID.Config(Config.rotationP, Config.rotationI, Config.rotationD);
        Movement movement = new Movement(robot.drive, this::opModeIsActive, translationConfig, rotationConfig, 1, telemetry);
        waitForStart();
        robot.drive.update();
        if (opModeIsActive()) {
            movement.move(new Pose2d(Config.targetX, Config.targetY, Math.toRadians(Config.targetH)));
            for (int i = 0; i < Config.pathCount; i ++) {
                movement.move(new Pose2d(20, 0, Math.toRadians(0)));
            }
            robot.drive.update();
        }
        while (opModeIsActive()) {
            telemetry.addData("Pose", robot.drive.getPos().toString());
            telemetry.update();
        }
    }
}