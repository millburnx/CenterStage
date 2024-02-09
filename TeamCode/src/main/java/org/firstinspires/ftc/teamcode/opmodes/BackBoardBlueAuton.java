package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ObjectDetector;
import org.firstinspires.ftc.teamcode.common.commands.AutonSeqBackBoardBlue1;
import org.firstinspires.ftc.teamcode.common.commands.AutonSeqBackBoardBlue1_5;
import org.firstinspires.ftc.teamcode.common.commands.AutonSeqBackBoardBlue2;
import org.firstinspires.ftc.teamcode.common.commands.BackBoardBlue;
import org.firstinspires.ftc.teamcode.common.commands.BlockerCommand;
import org.firstinspires.ftc.teamcode.common.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.IntakeUpCommand;
import org.firstinspires.ftc.teamcode.common.commands.LiftCommandBase;
import org.firstinspires.ftc.teamcode.common.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.common.commands.UpAndDeposit;
import org.firstinspires.ftc.teamcode.common.commands.DepositCommandBase;
import org.firstinspires.ftc.teamcode.common.drive.Drive;
import org.firstinspires.ftc.teamcode.common.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.common.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.common.subsystems.Intake;
import org.firstinspires.ftc.teamcode.common.subsystems.Lift;
import org.firstinspires.ftc.teamcode.common.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.common.utils.SubsystemsHardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
@TeleOp(name = "BackBoardBlueAuton")
public class BackBoardBlueAuton extends CommandOpMode {
    private final SubsystemsHardware subsystems = SubsystemsHardware.getInstance();
    private SampleMecanumDrive drive;
    private Intake intake;
    private Lift lift;
    private Deposit deposit;
    private Blocker blocker;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    ObjectDetector detector;
    GamepadEx gamepadEx;
    int region = 0;
    boolean end;
    boolean end2;
    Trajectory traj1, traj2, traj3, traj4;
    MecanumDriveSubsystem robot;

    double[] positions;
    SequentialCommandGroup auton1;


    @Override
    public void initialize() {
        end = true;
        end2 = false;
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        subsystems.init(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        robot = new MecanumDriveSubsystem(drive, false);
        intake = new Intake(subsystems);
        lift = new Lift(subsystems);
        deposit = new Deposit(subsystems);
        blocker = new Blocker(subsystems);
        detector = new ObjectDetector(hardwareMap, telemetry);


        subsystems.enabled = true;
        deposit.update(Deposit.DepositState.INTAKE);
        lift.update(Lift.LiftStates.DOWN);

        while(opModeInInit()){
            region = detector.getRegion();
//            telemetry.addLine(Integer.toString(region));
//            telemetry.update();
        }

    }
    public SequentialCommandGroup getAutonomousCommand1(Trajectory trajj1, Trajectory trajj2,Trajectory trajj3, Deposit deposit, Blocker blocker, Lift lift, Telemetry telemetry) {
        return new SequentialCommandGroup( //
//                new AutonSeqBackBoardBlue1(drive, region, telemetry),
//                new AutonSeqBackBoardBlue2(drive, region),
                //new DepositCommandBase(deposit, Deposit.DepositState.DEPOSIT2, telemetry),
                new TrajectoryFollowerCommand(robot, trajj1, telemetry),
                new IntakeUpCommand(intake, 1).withTimeout(1000),
                //new LiftCommandBase(lift,Lift.LiftStates.POS2),
                new TrajectoryFollowerCommand(robot, trajj2, telemetry),
                new UpAndDeposit(lift, deposit,blocker, -1, telemetry),
                new BlockerCommand(blocker, Blocker.BlockerState.RELEASE, telemetry),
                new TrajectoryFollowerCommand(robot, trajj3, telemetry),
                new UpAndDeposit(lift, deposit,blocker, 0, telemetry)


                );
    }
    public SequentialCommandGroup getAutonomousCommand2(Trajectory trajj1, Trajectory trajj2, Deposit deposit, Blocker blocker, Lift lift, Telemetry telemetry) {
        return new SequentialCommandGroup( //
//                new AutonSeqBackBoardBlue1(drive, region, telemetry),
//                new AutonSeqBackBoardBlue2(drive, region),
                //new DepositCommandBase(deposit, Deposit.DepositState.DEPOSIT2, telemetry),
                new TrajectoryFollowerCommand(robot, trajj1, telemetry),
                new UpAndDeposit(lift, deposit,blocker, -1, telemetry),
                new BlockerCommand(blocker, Blocker.BlockerState.RELEASE, telemetry),
                new TrajectoryFollowerCommand(robot, trajj2, telemetry),
                new UpAndDeposit(lift, deposit,blocker, 0, telemetry)


        );
    }



    @Override
    public void run() {
        super.run();
        robot.update();
        lift.loop();
        deposit.loop();
        blocker.loop();
        telemetry.addData("target depo", deposit.getTarget());
        telemetry.addData("curr depo", deposit.getPosition());
        telemetry.addData("cond",  Math.abs(deposit.getTarget()-deposit.getPosition())<0.01);

        positions = getPosition(region);

        if (end) {
            end = false;
            traj1 = drive.trajectoryBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(29, 0, Math.toRadians(-90)))
//                    .addDisplacementMarker(() -> {
//                        schedule(new IntakeCommand(intake, Intake.IntakeState.AUTON_OUT));
//                        schedule(new UpAndDeposit(lift, deposit, blocker, 1, telemetry));
//                    })
                    .build();
            //drive.followTrajectory(traj1);

            traj2 = drive.trajectoryBuilder(traj1.end())
                    .lineToLinearHeading(new Pose2d(33, 33, Math.toRadians(-90)))
//                    .addDisplacementMarker(() -> {
//                        schedule(new DepositCommandBase(deposit, Deposit.DepositState.DEPOSIT1, telemetry));
//                    })
                    .build();
            //drive.followTrajectory(traj2);


            traj3 = drive.trajectoryBuilder(traj2.end())
                    .forward(4)
                    .build();

            auton1=getAutonomousCommand1(traj1, traj2, traj3, deposit, blocker, lift, telemetry);
            schedule(auton1);
            end2 = true;
            initAprilTag();
        }

        if(end2 && auton1.isFinished()) {
            if (positions.length > 0 && positions[0] != 0) {
                traj1 = robot.trajectoryBuilder(new Pose2d())
                        .lineToLinearHeading(new Pose2d(-positions[1] + 10, positions[0] + 3, Math.toRadians(positions[2])))
                        .build();
                traj2 = robot.trajectoryBuilder(traj1.end())
                        .forward(4)
                        .build();
                schedule(getAutonomousCommand2(traj1, traj2, deposit, blocker, lift, telemetry));
                end2 = false;


            }
        }
        sleep(20);




        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();
    }


    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

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
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"));
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
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()

    public double[] getPosition(int id) {
        double[] stoof = new double[3];
        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if(detection.id==id){
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