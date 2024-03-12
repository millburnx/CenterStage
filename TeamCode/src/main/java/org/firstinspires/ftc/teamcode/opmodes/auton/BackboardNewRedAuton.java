package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.common.commands.DepositCommandBase;
import org.firstinspires.ftc.teamcode.common.commands.HookCommand;
import org.firstinspires.ftc.teamcode.common.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.drive.DriveConstants;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ObjectDetector;
import org.firstinspires.ftc.teamcode.common.commands.BlockerCommand;
import org.firstinspires.ftc.teamcode.common.commands.IntakeUpCommand;
import org.firstinspires.ftc.teamcode.common.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.common.commands.UpAndDeposit;
import org.firstinspires.ftc.teamcode.common.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.common.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.common.subsystems.Hook;
import org.firstinspires.ftc.teamcode.common.subsystems.Intake;
import org.firstinspires.ftc.teamcode.common.subsystems.Lift;
import org.firstinspires.ftc.teamcode.common.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.common.utils.SubsystemsHardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
@TeleOp(name = "BackboardNewRedAuton")
public class BackboardNewRedAuton extends CommandOpMode {
    private final SubsystemsHardware subsystems = SubsystemsHardware.getInstance();
    private SampleMecanumDrive drive;
    private Intake intake;
    private Hook hook;
    private Lift lift;
    private Deposit deposit;
    private Blocker blocker;
    double lastX;
    double lastY;
    double lastH;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag1;
    private AprilTagProcessor aprilTag2;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    ObjectDetector detector;
    GamepadEx gamepadEx;
    int region = 0;
    boolean end;
    boolean end2;
    boolean end3;
    boolean inEnd;
    boolean inEnd2;

    double aprilTx;
    double aprilTy;

    Trajectory traj1,traj1_1,traj1_2, traj2, traj3, traj1pt2, traj1pt2_2,traj2pt2, traj2pt3, traj2pt4, boardTraj1, boardTraj2;
    MecanumDriveSubsystem robot;

    double[] positions;
    double[] positions2;

    SequentialCommandGroup auton1;
    double xEnd;
    double dEnd;
    double offset;



    @Override
    public void initialize() {
        end = true;
        end2 = false;
        end3 = false;
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        subsystems.init(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        robot = new MecanumDriveSubsystem(drive, false);
        intake = new Intake(subsystems);
        hook = new Hook(subsystems);
        lift = new Lift(subsystems);
        deposit = new Deposit(subsystems);
        blocker = new Blocker(subsystems);
        detector = new ObjectDetector(hardwareMap, telemetry);
        inEnd = false;


        subsystems.enabled = true;
        deposit.update(Deposit.DepositState.INTAKE);
        intake.updatePosition(0);
        lift.update(Lift.LiftStates.DOWN);

        while (opModeInInit()) {
            region = detector.getRegion();
            telemetry.addLine(Integer.toString(region));
            telemetry.update();
        }

    }
    public SequentialCommandGroup getAutonomousCommand1(Trajectory trajj1, Trajectory trajj2, Deposit deposit, Blocker blocker, Lift lift, Telemetry telemetry) {
        return new SequentialCommandGroup(
                new IntakeUpCommand(intake, 0).withTimeout(1000),
                new TrajectoryFollowerCommand(robot, trajj1, telemetry),
                new IntakeUpCommand(intake, 0.2).withTimeout(1000),
                new TrajectoryFollowerCommand(robot, trajj2, telemetry)

        );
    }

    public SequentialCommandGroup getAutonomousCommand1Alt(Trajectory trajj1,Trajectory trajj1_1, Trajectory trajj1_2, Trajectory trajj2, Deposit deposit, Blocker blocker, Lift lift, Telemetry telemetry) {
        return new SequentialCommandGroup(
                new TrajectoryFollowerCommand(robot, trajj1, telemetry),
                new TrajectoryFollowerCommand(robot, trajj1_1, telemetry),
                new IntakeUpCommand(intake, 0.16),
                new TrajectoryFollowerCommand(robot, trajj1_2, telemetry),
                new TrajectoryFollowerCommand(robot, trajj2, telemetry),
                new IntakeUpCommand(intake, 0)

        );
    }

    public SequentialCommandGroup getAutonomousCommand2(Trajectory trajj1, Trajectory traj1pt2_2, Trajectory trajj2,Trajectory trajj3, Trajectory traj2pt4, Deposit deposit, Blocker blocker, Lift lift, Telemetry telemetry) {
        return new SequentialCommandGroup(
                new IntakeCommand(intake, Intake.IntakeState.IN),
                new IntakeUpCommand(intake, 0.16),
                new TrajectoryFollowerCommand(robot, trajj1, telemetry),
                new TrajectoryFollowerCommand(robot, traj1pt2_2, telemetry),
                new IntakeUpCommand(intake, 0.077),
                new TrajectoryFollowerCommand(robot, trajj2, telemetry),
                new TrajectoryFollowerCommand(robot, trajj3, telemetry),
                new IntakeCommand(intake, Intake.IntakeState.OUT),
                new TrajectoryFollowerCommand(robot, traj2pt4, telemetry)

                );
    }

    public SequentialCommandGroup getAutonomousCommand3(Trajectory trajj1, Trajectory trajj2, Deposit deposit, Blocker blocker, Lift lift, Telemetry telemetry) {
        return new SequentialCommandGroup(
                new UpAndDeposit(lift, deposit,blocker, hook,-1, telemetry),
                new TrajectoryFollowerCommand(robot, trajj1, telemetry),
                new BlockerCommand(blocker, Blocker.BlockerState.RELEASE, telemetry),
                new DepositCommandBase(deposit, Deposit.DepositState.INTAKE, telemetry),
                new TrajectoryFollowerCommand(robot, trajj2, telemetry),
                new UpAndDeposit(lift, deposit,blocker, hook,0, telemetry)

        );
    }



    @Override
    public void run() {
        super.run();
        robot.update();
        lift.loop();
        deposit.loop();
        blocker.loop();
        intake.loop();
        telemetry.addData("robot x: ", robot.getPoseEstimate().getX());
        telemetry.addData("robot y: ", robot.getPoseEstimate().getY());
        telemetry.addData("robot heading: ", robot.getPoseEstimate().getHeading());
        telemetry.addData("robot busy: ", drive.isBusy());

        if (end) {
            end = false;
            if (region==0) {
                traj1 = drive.trajectoryBuilder(new Pose2d())
                        .lineToLinearHeading(new Pose2d(29, 0, Math.toRadians(-90)))
                        .build();
                traj1_1 = drive.trajectoryBuilder(traj1.end())
                        .forward(4)
                        .build();
                traj1_2 = drive.trajectoryBuilder(traj1_1.end())
                        .back(4)
                        .build();
                offset =-4.25;
                aprilTx = 30;
            } else if (region ==1) {
                traj1 = drive.trajectoryBuilder(new Pose2d())
                        .lineToLinearHeading(new Pose2d(30, -1, Math.toRadians(0)))
                        .build();
                offset =-2;
                aprilTx = 33;
            } else {
                traj1 = drive.trajectoryBuilder(new Pose2d())
                        .lineToLinearHeading(new Pose2d(31, 0, Math.toRadians(90)))
                        .build();
                offset = -1;
                aprilTx = 36;
            }


            if(region==0){
                traj2 = drive.trajectoryBuilder(traj1_2.end())
                        .lineToLinearHeading(new Pose2d(25, 6, Math.toRadians(90)))
                        .build();

                auton1=getAutonomousCommand1Alt(traj1, traj1_1,traj1_2, traj2,  deposit, blocker, lift, telemetry);

            }
            else{
                traj2 = drive.trajectoryBuilder(traj1.end())
                        .lineToLinearHeading(new Pose2d(25, 5, Math.toRadians(90)))
                        .build();
                auton1=getAutonomousCommand1(traj1, traj2, deposit, blocker, lift, telemetry);

            }
            schedule(auton1);
            detector.close();
            initAprilTag1();
            end2 = true;
            xEnd = 25;
        }
        positions = getPosition1(8);
        telemetry.addLine("STARTED SECOND STAGE");
        telemetry.addData("apriltag x: ", positions[0]);
        telemetry.addData("apriltag y: ", positions[1]);
        telemetry.addData("apriltag heading: ", positions[2]);
        telemetry.addData("region: ", region);
        if ((end2 && !robot.isBusy() && Math.abs(robot.getPoseEstimate().getX()-xEnd)<5 && Math.abs(robot.getPoseEstimate().getY()-8)<5)||inEnd) {
            inEnd = true;
            positions = getPosition1(8);
            telemetry.addLine("STARTED SECOND STAGE");
            telemetry.addData("apriltag x: ", positions[0]);
            telemetry.addData("apriltag y: ", positions[1]);
            telemetry.addData("apriltag heading: ", positions[2]);
            telemetry.update();
            if (positions.length > 0 && positions[0] != 0) {
                traj1pt2 = robot.trajectoryBuilder(traj2.end())
                        .lineToLinearHeading(new Pose2d(xEnd+positions[0]+3.5, 8+positions[1]-3.5, Math.toRadians(90+positions[2])),
                                SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();
                traj1pt2_2 = robot.trajectoryBuilder(traj1pt2.end())
                        .back(2)
                        .build();
                lastX = xEnd+positions[0]+2.5;
                lastY = 8+positions[1]-3;
                lastH = Math.toRadians(90+positions[2]+2);
                traj2pt2 = robot.trajectoryBuilder(traj1pt2_2.end())
                        .lineToLinearHeading(new Pose2d(48, 0, 90))
                        .build();
                traj2pt3 = robot.trajectoryBuilder(traj2pt2.end())
                        .lineToLinearHeading(new Pose2d(48, -73, 90))
                        .build();
                traj2pt4 = robot.trajectoryBuilder(traj2pt3.end())
                        .lineToLinearHeading(new Pose2d(33, -73, 90))
                        .build();

                schedule(getAutonomousCommand2(traj1pt2,traj1pt2_2, traj2pt2, traj2pt3, traj2pt4,deposit, blocker, lift, telemetry));
                lastY = lastY-80;
                end2 = false;
                end3 = true;
                inEnd = false;
                visionPortal.close();
                initAprilTag2();
            }
        }

        if ((end3 && !robot.isBusy() && Math.abs(robot.getPoseEstimate().getX()-33)<5 && Math.abs(robot.getPoseEstimate().getY()-(-73))<5)||inEnd2) {
            inEnd2 = true;
            positions2 = getPosition2(6-region);
            if (positions2.length > 0 && positions2[0] != 0) {
                boardTraj1 = robot.trajectoryBuilder(traj2.end())
                        .lineToLinearHeading(new Pose2d(aprilTx+positions2[0]+offset, -73+positions2[1], Math.toRadians(90+positions2[2])),
                                SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();
                boardTraj2 = robot.trajectoryBuilder(traj1pt2.end())
                        .forward(4)
                        .build();

                schedule(getAutonomousCommand3(boardTraj1, boardTraj2, deposit, blocker, lift, telemetry));
                end2 = false;
                inEnd2 = false;
                // Save more CPU resources when camera is no longer needed.
                visionPortal.close();
            }
        }


        telemetry.update();
    }


    private void initAprilTag2() {

        // Create the AprilTag processor.
        aprilTag2 = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag2);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()

    private void initAprilTag1() {

        // Create the AprilTag processor.
        aprilTag1 = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag1);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }

    public double[] getPosition1(int id) {
        double[] stoof = new double[3];
        for (AprilTagDetection detection : aprilTag1.getDetections()) {
            if (detection.id==id) {
                stoof[0] = detection.ftcPose.x;
                stoof[1] = detection.ftcPose.y;
                stoof[2] = detection.ftcPose.yaw;
            }
        }
        return stoof;
    }

    public double[] getPosition2(int id) {
        double[] stoof = new double[3];
        for (AprilTagDetection detection : aprilTag1.getDetections()) {
            if (detection.id==id) {
                stoof[0] = detection.ftcPose.x;
                stoof[1] = detection.ftcPose.y;
                stoof[2] = detection.ftcPose.yaw;
            }
        }
        return stoof;
    }

    /**
     * Add telemetry about AprilTag detections.
     */










}