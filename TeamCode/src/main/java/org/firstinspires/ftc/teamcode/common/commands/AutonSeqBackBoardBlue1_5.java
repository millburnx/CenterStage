package org.firstinspires.ftc.teamcode.common.commands;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.common.subsystems.Lift;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.common.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;




public class AutonSeqBackBoardBlue1_5 extends CommandBase {
    public Trajectory left_0, left_1;
    public Trajectory middle_0, middle_1;
    public Trajectory right_0, right_1;

    double x;
    double y;



    private SampleMecanumDrive robotobj;
    private Deposit depositobj;
    private int pos;
    public AutonSeqBackBoardBlue1_5(SampleMecanumDrive robot, int position){

        pos = position;
        robotobj = robot;

    }

    @Override
    public void initialize(){
        x = 1000;
        y = 1000;
        if(pos==0){
            left_1 = robotobj.trajectoryBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(-33,4, Math.toRadians(0)))
                    .build();
            x = -33;
            y = 4;
            robotobj.followTrajectory(left_1);
        }
        else if(pos==1){
            middle_1 = robotobj.trajectoryBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(0, 30 , Math.toRadians(-90)))
                    .build();
            x = 0;
            y = 30;
            robotobj.followTrajectory(middle_1);
        }
        else{
            right_1 = robotobj.trajectoryBuilder(right_0.end())
                    .lineToLinearHeading(new Pose2d(-8, -12, Math.toRadians(0)))
                    .build();
            x = -8;
            y = -12;
            robotobj.followTrajectory(right_1);
        }

    }
    @Override
    public boolean isFinished(){
        if(Math.abs(robotobj.getPose().getX()-x) <1 && Math.abs(robotobj.getPose().getY()-y)<1){
            return true;
        }
        return false;
    }




}
