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




    private SampleMecanumDrive robotobj;
    private Deposit depositobj;
    private int pos;
    public AutonSeqBackBoardBlue1_5(SampleMecanumDrive robot, int position){

        pos = position;
        SampleMecanumDrive robotobj = robot;

    }

    @Override
    public void initialize(){
        if(pos==0){
            left_1 = robotobj.trajectoryBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(-33,4, Math.toRadians(0)))
                    .build();
            robotobj.followTrajectoryAsync(left_1);
        }
        else if(pos==1){
            middle_1 = robotobj.trajectoryBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(0, 30 , Math.toRadians(-90)))
                    .build();
            robotobj.followTrajectoryAsync(middle_1);
        }
        else{
            right_1 = robotobj.trajectoryBuilder(right_0.end())
                    .lineToLinearHeading(new Pose2d(-8, -12, Math.toRadians(0)))
                    .addTemporalMarker(2,()->{
                    } )
                    .build();
            robotobj.followTrajectoryAsync(right_1);
        }

    }




}
