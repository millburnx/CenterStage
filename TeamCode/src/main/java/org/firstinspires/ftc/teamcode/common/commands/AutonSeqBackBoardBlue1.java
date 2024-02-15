package org.firstinspires.ftc.teamcode.common.commands;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.common.subsystems.Lift;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.common.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;




public class AutonSeqBackBoardBlue1 extends CommandBase {
    public Trajectory left_0, left_1;
    public Trajectory middle_0, middle_1;
    public Trajectory right_0, right_1;
    double x;
    double y;


    Telemetry telemetry;
    private SampleMecanumDrive robotobj;

    private int pos;
    public AutonSeqBackBoardBlue1(SampleMecanumDrive robot, int position,Telemetry t) {
        pos = position;
        robotobj = robot;
        telemetry = t;
    }

    @Override
    public void initialize() {
        //telemetry.addLine("started");
        x = 10000;
        y = 10000;
        if (pos==0) {
            left_0 = robotobj.trajectoryBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(29, 0, Math.toRadians(-90)))
                    .build();
            x = 29;
            y = 9;
            robotobj.followTrajectory(left_0);
        }
        else if (pos==1) {
            middle_0 = robotobj.trajectoryBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(26,0,0))
                    .build();
            x = 26;
            y = 0;
            robotobj.followTrajectory(middle_0);
        }
        else {
            right_0 = robotobj.trajectoryBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(30, 25, Math.toRadians(-85)))
                    .build();
            x = 30;
            y = 25;
            robotobj.followTrajectory(right_0);
        }
        //telemetry.update();

    }
    @Override
    public void execute() {
        robotobj.update();
    }

    @Override
    public void end(boolean interrupted) {
        telemetry.addLine("end");
        telemetry.update();
    }
    @Override
    public boolean isFinished() {
        return !robotobj.isBusy();
    }


}
