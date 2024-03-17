package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.AnalogInput;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.commands.Ultrasonic;
import org.firstinspires.ftc.teamcode.common.drive.DriveConstants;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ObjectDetector;
import org.firstinspires.ftc.teamcode.common.commands.BlockerCommand;
import org.firstinspires.ftc.teamcode.common.commands.IntakeUpCommand;
import org.firstinspires.ftc.teamcode.common.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.common.commands.UpAndDeposit;
import org.firstinspires.ftc.teamcode.common.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.common.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.common.subsystems.Intake;
import org.firstinspires.ftc.teamcode.common.subsystems.Lift;
import org.firstinspires.ftc.teamcode.common.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.common.utils.SubsystemsHardware;
import org.firstinspires.ftc.teamcode.common.utils.UltrasonicDistanceSensor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
@TeleOp(name = "UltrasonicTest")
public class UltrasonicTest extends CommandOpMode {
    private final SubsystemsHardware subsystems = SubsystemsHardware.getInstance();
    private SampleMecanumDrive drive;
    private Intake intake;
    private Lift lift;
    private Deposit deposit;
    private Blocker blocker;

    ObjectDetector detector;
    GamepadEx gamepadEx;
    int region = 0;
    boolean end;
    boolean end2;
    boolean inEnd;
    Trajectory traj1, traj2;
    MecanumDriveSubsystem robot;

    double[] positions;
    SequentialCommandGroup auton1;

    public UltrasonicDistanceSensor frontUltrasonic;
    public UltrasonicDistanceSensor backUltrasonic;

    public UltrasonicDistanceSensor diagUltrasonic;



    @Override
    public void initialize() {
        end = true;
        end2 = false;
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        subsystems.init(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        robot = new MecanumDriveSubsystem(drive, false);
        intake = new Intake(subsystems);
        lift = new Lift(subsystems);
        deposit = new Deposit(subsystems);
        blocker = new Blocker(subsystems);
        detector = new ObjectDetector(hardwareMap, telemetry);
        inEnd = false;
        frontUltrasonic = new UltrasonicDistanceSensor(hardwareMap.get(AnalogInput.class, "frontUltrasonic"));
        diagUltrasonic = new UltrasonicDistanceSensor(hardwareMap.get(AnalogInput.class, "diagUltrasonic"));
        backUltrasonic = new UltrasonicDistanceSensor(hardwareMap.get(AnalogInput.class, "backUltrasonic"));

        subsystems.enabled = true;
        deposit.update(Deposit.DepositState.INTAKE);
        lift.update(Lift.LiftStates.DOWN);

        while (opModeInInit()) {
            region = detector.getRegion();
            telemetry.addLine(Integer.toString(region));
            telemetry.update();
        }

    }
    public SequentialCommandGroup getAutonomousCommand1(Trajectory trajj1, Trajectory trajj2, Deposit deposit, Blocker blocker, Lift lift, Telemetry telemetry, UltrasonicDistanceSensor fUltra) {
        return new SequentialCommandGroup( //
                new TrajectoryFollowerCommand(robot, trajj1, telemetry),
                new Ultrasonic(fUltra),
                new TrajectoryFollowerCommand(robot, trajj2, telemetry)

        );
    }



    @Override
    public void run() {
        super.run();
        robot.update();
        lift.loop();
        deposit.loop();
        blocker.loop();

        if (end) {
            end = false;
            traj1 = drive.trajectoryBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(29, 0, Math.toRadians(0)))
                    .build();
            traj2 = drive.trajectoryBuilder(traj1.end())
                    .forward(10)
                    .build();
            auton1=getAutonomousCommand1(traj1, traj2, deposit, blocker, lift, telemetry, frontUltrasonic);
            schedule(auton1);
            detector.close();
        }
        telemetry.addData("Front Ultrasonic", frontUltrasonic.getDistance(DistanceUnit.INCH));
        telemetry.addData("Diag Ultrasonic", diagUltrasonic.getDistance(DistanceUnit.INCH));
        telemetry.update();
    }

    /**
     * Add telemetry about AprilTag detections.
     */
}