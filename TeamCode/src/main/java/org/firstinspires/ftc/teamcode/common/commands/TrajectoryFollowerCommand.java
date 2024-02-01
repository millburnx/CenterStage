package org.firstinspires.ftc.teamcode.common.commands;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.subsystems.MecanumDriveSubsystem;


public class TrajectoryFollowerCommand extends CommandBase {
    Telemetry telemetry;

    private final MecanumDriveSubsystem drive;
    private final Trajectory trajectory;

    public TrajectoryFollowerCommand(MecanumDriveSubsystem drive, Trajectory trajectory, Telemetry t) {
        this.drive = drive;
        this.trajectory = trajectory;
        telemetry = t;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.followTrajectory(trajectory);
    }

    @Override
    public void execute() {
        drive.update();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            drive.stop();
        }
    }

    @Override
    public boolean isFinished() {
        telemetry.addLine("finished");
        telemetry.update();
        return Thread.currentThread().isInterrupted() || !drive.isBusy();
    }

}