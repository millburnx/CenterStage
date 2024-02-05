package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ObjectDetector;
import org.firstinspires.ftc.teamcode.common.commands.AutonSeqBackBoardBlue1;
import org.firstinspires.ftc.teamcode.common.commands.AutonSeqBackBoardBlue1_5;
import org.firstinspires.ftc.teamcode.common.commands.AutonSeqBackBoardBlue2;
import org.firstinspires.ftc.teamcode.common.commands.BackBoardBlue;
import org.firstinspires.ftc.teamcode.common.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.common.commands.UpAndDeposit;
import org.firstinspires.ftc.teamcode.common.commands.DepositCommandBase;
import org.firstinspires.ftc.teamcode.common.drive.Drive;
import org.firstinspires.ftc.teamcode.common.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.common.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.common.subsystems.Intake;
import org.firstinspires.ftc.teamcode.common.subsystems.Lift;
import org.firstinspires.ftc.teamcode.common.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.common.utils.SubsystemsHardware;

@Config
@TeleOp(name = "BackBoardBlueAuton")
public class BackBoardBlueAuton extends CommandOpMode {
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
    Trajectory traj1, traj2, traj3, traj4;
    MecanumDriveSubsystem robot;

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
        detector = new ObjectDetector(hardwareMap, telemetry);

        subsystems.enabled = true;
        deposit.update(Deposit.DepositState.INTAKE);
        lift.update(Lift.LiftStates.DOWN);
        while(opModeInInit()){
            region = detector.getRegion();
//            telemetry.addLine(Integer.toString(region));
//            telemetry.update();
        }
    }
    public Command getAutonomousCommand(Trajectory trajj1, Trajectory trajj2) {

        return new SequentialCommandGroup( //
//                new AutonSeqBackBoardBlue1(drive, region, telemetry),
//                new AutonSeqBackBoardBlue2(drive, region)
                new TrajectoryFollowerCommand(robot, trajj1, telemetry),
                new TrajectoryFollowerCommand(robot, trajj2, telemetry)


                );
    }

    @Override
    public void run() {
        drive.update();
        if(end){
            traj1 = drive.trajectoryBuilder(new Pose2d())
                        .lineToLinearHeading(new Pose2d(29, 0, Math.toRadians(-90)))
                        .build();
            drive.followTrajectory(traj1);

            schedule(new IntakeCommand(intake, Intake.IntakeState.AUTON_OUT));
            traj2 = drive.trajectoryBuilder(traj1.end())
                    .lineToLinearHeading(new Pose2d(33,36, Math.toRadians(-90)))
                    .build();

            drive.followTrajectory(traj2);
            schedule(new UpAndDeposit(lift, deposit,blocker, 1, telemetry));

            traj3 = drive.trajectoryBuilder(traj2.end())
                    .forward(4)
                    .build();
            drive.followTrajectory(traj3);
            end = false;
        }
    }








}