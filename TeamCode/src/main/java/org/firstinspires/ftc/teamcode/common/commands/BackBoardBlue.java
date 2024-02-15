package org.firstinspires.ftc.teamcode.common.commands;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.common.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.common.subsystems.Intake;
import org.firstinspires.ftc.teamcode.common.subsystems.Lift;
import org.firstinspires.ftc.teamcode.common.subsystems.MecanumDriveSubsystem;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;




public class BackBoardBlue extends SequentialCommandGroup {
    public BackBoardBlue(Lift lift, Deposit deposit, Intake intake, MecanumDriveSubsystem robot, Trajectory traj, Telemetry telemetry) {

        addCommands(
                new DepositCommandBase(deposit, Deposit.DepositState.DEPOSIT2, telemetry),
                new TrajectoryFollowerCommand(robot, traj, telemetry)

        );


        addRequirements(lift, deposit);
    }
}
