package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.PixelDetector;
import org.firstinspires.ftc.teamcode.common.commands.IntakeCommand;
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
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
@TeleOp(name = "TestingPixelAuton")
public class TestingPixelAuton extends CommandOpMode {
    private final SubsystemsHardware subsystems = SubsystemsHardware.getInstance();
    private SampleMecanumDrive drive;
    private Intake intake;
    private Lift lift;
    private Deposit deposit;
    private Blocker blocker;
    private final double pixelToInches = 1;
    //TODO: find the conversion

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    PixelDetector detector;
    GamepadEx gamepadEx;
    int region = 0;
    boolean end;
    boolean end2;
    boolean inEnd;
    Trajectory traj1;
    MecanumDriveSubsystem robot;

    int x;




    @Override
    public void initialize() {
        end = true;
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        subsystems.init(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        robot = new MecanumDriveSubsystem(drive, false);
        intake = new Intake(subsystems);
        lift = new Lift(subsystems);
        deposit = new Deposit(subsystems);
        blocker = new Blocker(subsystems);
        detector = new PixelDetector(hardwareMap, telemetry);


        subsystems.enabled = true;
        deposit.update(Deposit.DepositState.INTAKE);
        lift.update(Lift.LiftStates.DOWN);

        while (opModeInInit()) {
            x = detector.getX();
            telemetry.addLine(Integer.toString(x));
            telemetry.update();
        }

    }
    public SequentialCommandGroup getAutonomousCommand1(Trajectory trajj1) {
        return new SequentialCommandGroup( //
                new IntakeCommand(intake, Intake.IntakeState.IN),
                new IntakeUpCommand(intake, 1).withTimeout(1000),
                new TrajectoryFollowerCommand(robot, trajj1, telemetry)

        );
    }

    @Override
    public void run() {
        super.run();
        robot.update();
        lift.loop();
        deposit.loop();
        blocker.loop();
        telemetry.addData("x: ", x*pixelToInches);

        if (end) {
            end = false;
            traj1 = drive.trajectoryBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(0, x*pixelToInches, Math.toRadians(0)))
                    .build();
            schedule(getAutonomousCommand1(traj1));
        }
        telemetry.addData("auton1 finished: ", robot.isBusy());
        telemetry.addData("region: ", region);

        telemetry.update();
    }
}