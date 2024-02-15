package org.firstinspires.ftc.teamcode.common.commands;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.common.subsystems.Lift;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.common.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;




public class AutonSeqBackBoardBlue2 extends CommandBase {
    Trajectory left_2, left_3;
    Trajectory middle_2, middle_3;

    double x;
    double y;

    Trajectory right_2, right_3;
    private SampleMecanumDrive robotobj;
    int pos;
    public AutonSeqBackBoardBlue2(SampleMecanumDrive robot, int position) {
        robotobj = robot;
        pos = position;

    }

    @Override
    public void initialize() {
        x = 1000;
        y = 1000;
        left_3 = robotobj.trajectoryBuilder(new Pose2d())
                .forward(4)
                .build();
        x =4;
        y = 0;
        robotobj.followTrajectory(left_3);

    }
    @Override
    public void execute() {
        robotobj.update();
    }


}
