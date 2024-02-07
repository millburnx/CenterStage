package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.ObjectDetector;
import org.firstinspires.ftc.teamcode.common.drive.Drive;
import org.firstinspires.ftc.teamcode.common.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.common.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.common.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.common.subsystems.Intake;
import org.firstinspires.ftc.teamcode.common.subsystems.Lift;
import org.firstinspires.ftc.teamcode.common.utils.SubsystemsHardware;

@Autonomous(name="BackboardLeftAuton", group="Autonomous")
public class BackboardLeftAuton extends OpMode {
    private int region;
    Trajectory left_0, left_1, left_2, left_2_5, left_3;
    Trajectory middle_0, middle_1, middle_2, middle_2_5, middle_3;
    Trajectory right_0, right_1, right_2, right_3, right_2_5, right_4;
    boolean activated = false;
    boolean outtaking = false;
    boolean up = false;
    boolean down = false;
    boolean deposit = false;

    ObjectDetector detector;

    SampleMecanumDrive robot;

    private final SubsystemsHardware subsystems = SubsystemsHardware.getInstance();
    private Drive drive;
    private Intake intake;
    private Lift lift;
    private Deposit depositobj;
    private Blocker blocker;

    @Override
    public void init(){
        detector = new ObjectDetector(hardwareMap, telemetry);
        subsystems.init(hardwareMap);
        drive = new Drive(hardwareMap);
        intake = new Intake(subsystems);
        lift = new Lift(subsystems);
        depositobj = new Deposit(subsystems);
        blocker = new Blocker(subsystems);
    }

    @Override
    public void init_loop(){
        region = detector.getRegion();
        telemetry.addLine(Integer.toString(region));
        telemetry.update();
    }

    @Override
    public void start(){


        robot = new SampleMecanumDrive(hardwareMap);

        left_0 = robot.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(29, 0, Math.toRadians(-90)))
                .addTemporalMarker(4, () -> {
                    outtaking = true;
                    robot.followTrajectoryAsync(left_1);
                })
                .build();
        left_1 = robot.trajectoryBuilder(left_0.end())
                .lineToLinearHeading(new Pose2d(33,33, Math.toRadians(-90)))
                .addTemporalMarker(4, ()->{
                    up = true;
                    deposit = true;
                    robot.followTrajectoryAsync(left_2);
                })
                .build();
        left_2 = robot.trajectoryBuilder(left_1.end())
                .lineToLinearHeading(new Pose2d(33, 36, Math.toRadians(-90)),
                        SampleMecanumDrive.getVelocityConstraint(2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(2,()->{
                    up = false;
                    deposit = false;
                    robot.followTrajectoryAsync(left_2_5);
                } )
                .build();
        left_2_5 = robot.trajectoryBuilder(left_2.end())
                .lineToLinearHeading(new Pose2d(33, 36.01, Math.toRadians(-90)))
                .addTemporalMarker(2,()->{
                    down = true;
                    robot.followTrajectoryAsync(left_3);
                } )
                .build();
        left_3 = robot.trajectoryBuilder(left_2_5.end())
                .forward(4)
                .addTemporalMarker(3,()->{
                    down = false;
                } )
                .build();

        middle_0 = robot.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(26,0,0))
                .addTemporalMarker(4, () -> {
                    outtaking = true;
                    robot.followTrajectoryAsync(middle_1);
                })
                .build();
        middle_1 = robot.trajectoryBuilder(middle_0.end())
                .lineToLinearHeading(new Pose2d(26, 30 , Math.toRadians(-90)))
                .addTemporalMarker(4, ()->{
                    up = true;
                    deposit = true;
                    robot.followTrajectoryAsync(middle_2);
                })
                .build();
        middle_2 = robot.trajectoryBuilder(middle_1.end())
                .lineToLinearHeading(new Pose2d(26, 37.5, Math.toRadians(-90)),
                        SampleMecanumDrive.getVelocityConstraint(4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(2,()->{
                    up = false;
                    deposit = false;
                    robot.followTrajectoryAsync(middle_2_5);
                } )
                .build();
        middle_2_5 = robot.trajectoryBuilder(middle_2.end())
                .lineToLinearHeading(new Pose2d(26, 37.51, Math.toRadians(-90)))
                .addTemporalMarker(2,()->{
                    down = true;
                    robot.followTrajectoryAsync(middle_3);
                } )
                .build();
        middle_3 = robot.trajectoryBuilder(middle_2_5.end())
                .forward(4)
                .addTemporalMarker(3,()->{
                    down = false;
                } )
                .build();


        right_0 = robot.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(30, 25, Math.toRadians(-80)))
                .addTemporalMarker(3, () -> {
                    intake.update(Intake.IntakeState.AUTON_OUT);
                    robot.followTrajectoryAsync(right_1);
                })
                .build();

        right_1 = robot.trajectoryBuilder(right_0.end())
                .lineToLinearHeading(new Pose2d(18, 33, Math.toRadians(-85)))
                .addTemporalMarker(2,()->{
//                    lift.update(Lift.LiftStates.POS1);
                    up = true;

                    depositobj.update(Deposit.DepositState.DEPOSIT1);
                    robot.followTrajectoryAsync(right_2);
                } )
                .build();
        right_2 = robot.trajectoryBuilder(right_1.end())
                .lineToLinearHeading(new Pose2d(18, 35.5, Math.toRadians(-85)),
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(1,()->{
                    depositobj.update(Deposit.DepositState.INTAKE);
                    lift.update(Lift.LiftStates.DOWN);
                    robot.followTrajectoryAsync(right_2_5);
                } )
                .build();
        right_2_5 = robot.trajectoryBuilder(right_2.end())
                .lineToLinearHeading(new Pose2d(18, 35.55, Math.toRadians(-85)),
                        SampleMecanumDrive.getVelocityConstraint(0.5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(1,()->{
//                    down = true;
                    robot.followTrajectoryAsync(right_3);
                } )
                .build();
        right_3 = robot.trajectoryBuilder(right_2_5.end())
                .forward(4)
                .addTemporalMarker(2,()->{
//                    down = false;
                } )
                .build();

        // auton goes here
        if  (region == 0) {
            robot.followTrajectoryAsync(left_0);
        } else if (region == 1) {
            robot.followTrajectoryAsync(middle_0);
        } else {
            robot.followTrajectoryAsync(right_0);
        }

        activated = true;
        telemetry.addData("activated: ", activated);
    }

    public void intakeAsync(){
        if(outtaking){
            intake.update(Intake.IntakeState.AUTON_OUT);
        }
        else{
            intake.update(Intake.IntakeState.IN);
        }
    }

    public void depositAsync(){
        if(deposit){
            depositobj.update(Deposit.DepositState.DEPOSIT2);
        }
        else{
            depositobj.update(Deposit.DepositState.INTAKE);
        }
    }
    public void liftAsync(){
        if(up){
            lift.target = 500;

        }
        else if(down){
            lift.update(Lift.LiftStates.DOWN);
        }
        else{
            lift.update(Lift.LiftStates.DOWN);

        }
    }
    @Override
    public void loop(){
        if(activated){
            telemetry.addData("target", lift.target);
            lift.loop();
            depositobj.loop();
//            intakeAsync();
            liftAsync();
//            depositAsync();
            robot.update();
            telemetry.addLine("loop");
            telemetry.update();
        }
    }
}