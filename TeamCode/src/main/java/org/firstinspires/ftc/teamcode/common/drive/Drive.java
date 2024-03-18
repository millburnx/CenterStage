package org.firstinspires.ftc.teamcode.common.drive;

import static org.firstinspires.ftc.teamcode.common.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.common.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.common.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.common.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.common.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.common.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.common.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.common.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.common.drive.DriveConstants.kV;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.common.utils.UltrasonicDistanceSensor;



import org.firstinspires.ftc.teamcode.common.utils.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.common.utils.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.common.utils.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.common.utils.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class Drive extends MecanumDrive {
    public boolean isFieldCentric;
    public BNO055IMU imu;
    public MotorEx leftFront;
    public MotorEx leftRear;
    public MotorEx rightFront;
    public MotorEx rightRear;
    private List<MotorEx> motors;
    public double x, y, heading;

    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);
    public static double LATERAL_MULTIPLIER = 1;
    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private TrajectoryFollower follower;

    private VoltageSensor batteryVoltageSensor;

    private List<Integer> lastEncPositions = new ArrayList<>();
    private List<Integer> lastEncVels = new ArrayList<>();

    public static final double TRACKWIDTH = 12;
    public static final double CENTER_WHEEL_OFFSET = 0; // distance between center of rotation of the robot and the center odometer
    public static final double WHEEL_DIAMETER = 2.0;
    public static final double TICKS_PER_REV = 8192;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    public Motor.Encoder leftOdom, rightOdom, centerOdom;
    public HolonomicOdometry odometry;
    public UltrasonicDistanceSensor leftUltrasonic;
    public UltrasonicDistanceSensor rightUltrasonic;
    private TrajectorySequence queuedTrajectorySequence;


    public Drive(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        leftFront = new MotorEx(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_435);
        leftRear = new MotorEx(hardwareMap, "backLeft", Motor.GoBILDA.RPM_435);
        rightFront = new MotorEx(hardwareMap, "frontRight", Motor.GoBILDA.RPM_435);
        rightRear = new MotorEx(hardwareMap, "backRight", Motor.GoBILDA.RPM_435);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (MotorEx motor : motors) {
            motor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
            motor.set(0);
            motor.resetEncoder();
            motor.motorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        leftFront.motorEx.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.motorEx.setDirection(DcMotorSimple.Direction.REVERSE);

        x = 0;
        y = 0;
        heading = 0;

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        trajectorySequenceRunner = new TrajectorySequenceRunner(
                follower, HEADING_PID, batteryVoltageSensor,
                lastEncPositions, lastEncVels, lastTrackingEncPositions, lastTrackingEncVels
        );

        // odom stuff

        leftOdom = rightFront.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightOdom = rightRear.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        centerOdom = leftRear.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        rightOdom.setDirection(Motor.Direction.REVERSE);
        leftOdom.setDirection(Motor.Direction.REVERSE);

        leftOdom.reset();
        rightOdom.reset();
        centerOdom.reset();

        odometry = new HolonomicOdometry(
                leftOdom::getDistance,
                rightOdom::getDistance,
                centerOdom::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );

        // change to reflect starting field position
        odometry.updatePose(new com.arcrobotics.ftclib.geometry.Pose2d(0, 0, new Rotation2d(0)));

        leftUltrasonic = new UltrasonicDistanceSensor(hardwareMap.get(AnalogInput.class, "leftUltrasonic"));
        rightUltrasonic = new UltrasonicDistanceSensor(hardwareMap.get(AnalogInput.class, "rightUltrasonic"));

    }
    public void auton() {
        rightRear.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
    }

    public Pose2d getPos() {
        return new Pose2d(odometry.getPose().getX(), odometry.getPose().getY(), odometry.getPose().getHeading());
    }

    public void changeFollowerAccuracy(double timeout, double translational_error, double turn_error) {
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID, new Pose2d(translational_error, translational_error, Math.toRadians(turn_error)), timeout);
    }

    public void moveTeleOp(double power, double strafe, double turn) {
        if (isFieldCentric) {
            fieldCentric(power, strafe, turn);
        } else {
            robotCentric(power, strafe, turn);
        }
    }

    public void robotCentric(double power, double strafe, double turn) {
        double denominator = Math.max(Math.abs(power) + Math.abs(strafe) + Math.abs(turn), 1);
        double frontLeftPower = (power + strafe + turn) / denominator;
        double backLeftPower = (power - strafe + turn) / denominator;
        double frontRightPower = (power - strafe - turn) / denominator;
        double backRightPower = (power + strafe - turn) / denominator;

        leftFront.set(frontLeftPower);
        leftRear.set(backLeftPower);
        rightFront.set(frontRightPower);
        rightRear.set(backRightPower);
    }

    public void fieldCentric(double power, double strafe, double turn) {
        double botHeading = 0;
        double rotationX = strafe * Math.cos(botHeading) - power * Math.sin(botHeading);
        double rotationY = strafe * Math.sin(botHeading) + power * Math.cos(botHeading);
        double denominator = Math.max(Math.abs(power) + Math.abs(strafe) + Math.abs(turn), 1);
        double frontLeftPower = (rotationY + rotationX + turn) / denominator;
        double backLeftPower = (rotationY - rotationX + turn) / denominator;
        double frontRightPower = (rotationY - rotationX - turn) / denominator;
        double backRightPower = (rotationY + rotationX - turn) / denominator;

        leftFront.set(frontLeftPower);
        leftRear.set(backLeftPower*1.25);
        rightFront.set(frontRightPower);
        rightRear.set(backRightPower*1.25);
    }

    public void setDrivePowers(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        leftFront.set(frontLeftPower);
        leftRear.set(backLeftPower);
        rightFront.set(frontRightPower);
        rightRear.set(backRightPower);
    }

    public void switchDrive() {
        isFieldCentric = !isFieldCentric;
    }

    // ROADRUNNER METHODS

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double maxVelo, double maxAccel) {
        MinVelocityConstraint myVelConstraint = new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAccel),
                new MecanumVelocityConstraint(maxVelo, TRACK_WIDTH)
        ));
        ProfileAccelerationConstraint myAccelConstraint = new ProfileAccelerationConstraint(maxAccel);
        return new TrajectoryBuilder(startPose, myVelConstraint, myAccelConstraint);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPos()) // change this
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {

        updatePoseEstimate();
        odometry.updatePose();
        DriveSignal signal = trajectorySequenceRunner.update(getPos(), getPoseVelocity()); // change this
        if (signal != null) setDriveSignal(signal);

    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (MotorEx motor : motors) {
            motor.motorEx.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (MotorEx motor : motors) {
            motor.motorEx.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (MotorEx motor : motors) {
            motor.motorEx.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (MotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.motorEx.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (MotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.motorEx.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.set(v);
        leftRear.set(v1);
        rightRear.set(v2);
        rightFront.set(v3);
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return (double) imu.getAngularVelocity().xRotationRate;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
//        return 0;
    }

}
