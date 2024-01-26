package org.firstinspires.ftc.teamcode.common.commands;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.common.subsystems.Lift;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.common.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;




public class AutonSeqBackBoardBlue1 extends CommandBase {
    public Trajectory left_0, left_1;
    public Trajectory middle_0, middle_1;
    public Trajectory right_0, right_1;



    private SampleMecanumDrive robotobj;
    private int pos;
    public AutonSeqBackBoardBlue1(SampleMecanumDrive robot, int position){
        pos = position;
        SampleMecanumDrive robotobj = robot;
    }

    @Override
    public void initialize(){
        if(pos==0){
            left_0 = robotobj.trajectoryBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(29, 0, Math.toRadians(-90)))
                    .build();
            robotobj.followTrajectoryAsync(left_0);
        }
        else if(pos==1){
            middle_0 = robotobj.trajectoryBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(26,0,0))
                    .build();
            robotobj.followTrajectoryAsync(middle_0);
        }
        else{
            right_0 = robotobj.trajectoryBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(30, 25, Math.toRadians(-85)))
                    .build();
            robotobj.followTrajectoryAsync(right_0);
        }
    }


}
