package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ObjectDetector;
import org.firstinspires.ftc.teamcode.common.commands.IntakeUpCommand;
import org.firstinspires.ftc.teamcode.common.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.common.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.common.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.common.subsystems.Intake;
import org.firstinspires.ftc.teamcode.common.subsystems.Lift;
import org.firstinspires.ftc.teamcode.common.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.common.utils.SubsystemsHardware;

@Config
@Autonomous(name = "BasicBlueCloseAuton")
public class BasicBlueCloseAuton extends CommandOpMode {
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
    Trajectory traj1,traj1_1, traj2, traj3, traj1pt2, traj2pt2, traj2pt3, traj2pt4;
    MecanumDriveSubsystem robot;

    double[] positions;
    SequentialCommandGroup auton1;
    double xEnd;
    double yEnd;
    double offset;



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


        subsystems.enabled = true;
        deposit.update(Deposit.DepositState.INTAKE);
        lift.update(Lift.LiftStates.DOWN);

        while (opModeInInit()) {
            region = detector.getRegion();
            telemetry.addLine(Integer.toString(region));
            telemetry.update();
        }

    }
    public SequentialCommandGroup getAutonomousCommand1(Trajectory trajj1, Trajectory trajj2, Deposit deposit, Blocker blocker, Lift lift, Telemetry telemetry) {
        return new SequentialCommandGroup( //
                new TrajectoryFollowerCommand(robot, trajj1, telemetry),
                new IntakeUpCommand(intake, 0.3).withTimeout(1000),
                new TrajectoryFollowerCommand(robot, trajj2, telemetry)

        );
    }
    public SequentialCommandGroup getAutonomousCommand1Alt(Trajectory trajj1,Trajectory trajj1_1, Trajectory trajj2, Deposit deposit, Blocker blocker, Lift lift, Telemetry telemetry) {
        return new SequentialCommandGroup( //
                new TrajectoryFollowerCommand(robot, trajj1, telemetry),
                new TrajectoryFollowerCommand(robot, trajj1_1, telemetry),
                new IntakeUpCommand(intake, 0.3).withTimeout(1000),
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
        telemetry.addData("robot x: ", robot.getPoseEstimate().getX());
        telemetry.addData("robot y: ", robot.getPoseEstimate().getY());
        telemetry.addData("robot heading: ", robot.getPoseEstimate().getHeading());
        telemetry.addData("robot busy: ", drive.isBusy());

        if (end) {
            end = false;
            if (region==0) {
                traj1 = drive.trajectoryBuilder(new Pose2d())
                        .lineToLinearHeading(new Pose2d(29, 0, Math.toRadians(-87)))
                        .build();
                traj1_1 = drive.trajectoryBuilder(traj1.end())
                        .forward(3)
                        .build();
                xEnd = 32;
                offset = 5;
            } else if (region ==1) {
                traj1 = drive.trajectoryBuilder(new Pose2d())
                        .lineToLinearHeading(new Pose2d(33, 4, Math.toRadians(3)))
                        .build();
                xEnd = 24;
                offset = 2.75;
            } else {
                traj1 = drive.trajectoryBuilder(new Pose2d())
                        .lineToLinearHeading(new Pose2d(30, 3, Math.toRadians(90)))
                        .build();
                xEnd = 19;
                offset = 1;
            }



            if(region==0){
                traj2 = drive.trajectoryBuilder(traj1_1.end())
                        .back(8)
                        .build();
                auton1=getAutonomousCommand1Alt(traj1, traj1_1, traj2, deposit, blocker, lift, telemetry);
            }
            else{
                traj2 = drive.trajectoryBuilder(traj1.end())
                        .back(8)
                        .build();
                auton1=getAutonomousCommand1(traj1, traj2, deposit, blocker, lift, telemetry);
            }
            schedule(auton1);
            detector.close();
            end2 = true;
        }
        telemetry.addData("auton1 finished: ", robot.isBusy());
        telemetry.addData("region: ", region);
        telemetry.update();
    }

    /**
     * Add telemetry about AprilTag detections.
     */
}