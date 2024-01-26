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

    Trajectory right_2, right_3;
    private SampleMecanumDrive robotobj;
    int pos;
    public AutonSeqBackBoardBlue2(SampleMecanumDrive robot, int position){
        SampleMecanumDrive robotobj = robot;
        pos = position;

    }

    @Override
    public void initialize(){
        if(pos == 0){
            left_3 = robotobj.trajectoryBuilder(new Pose2d())
                    .forward(4)
                    .build();
            robotobj.followTrajectoryAsync(left_3);
        }
        else if(pos==1){
            middle_3 = robotobj.trajectoryBuilder(new Pose2d())
                    .forward(4)
                    .build();
            robotobj.followTrajectoryAsync(middle_3);
        }
        else{
            right_3 = robotobj.trajectoryBuilder(new Pose2d())
                    .forward(4)
                    .build();
            robotobj.followTrajectoryAsync(right_3);
        }
    }


}
