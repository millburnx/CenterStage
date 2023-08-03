package org.firstinspires.ftc.teamcode.ctrl;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.opencv.core.Point;

public class MoveTo {
    private Robot robot;
    private FtcDashboard dashboard;
    private Telemetry telemetry;
    public boolean isDone;
    private double currDistance, headingError;
    private final ElapsedTime profileTime = new ElapsedTime();
    private MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(0, 0, 0), new MotionState(1, 0, 0), 2, 3, 4);

    public MoveTo(Robot robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public void point(Pose2d pose, Pose2d desiredPose, Pose2d startPose, boolean update) {
        currDistance = Math.abs(Math.hypot(desiredPose.getX() - pose.getX(), desiredPose.getY() - pose.getY()));
        double distanceAtStart = Math.abs(Math.hypot(desiredPose.getX() - startPose.getX(), desiredPose.getY() - startPose.getY()));
        double distance = distanceAtStart - currDistance;

        double angle = Math.atan2(desiredPose.getY() - startPose.getY(), desiredPose.getX() - startPose.getX());

        if(update) {
            profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(0, 0, 0), new MotionState(distanceAtStart, 0, 0), 30, 50, 70);
            profileTime.reset();
        }
        MotionState state = profile.get(profileTime.seconds());
        double stateOut = state.getX();
        Point statePoint = new Point(startPose.getX() + (stateOut * Math.cos(angle)), startPose.getY() + (stateOut * Math.sin(angle)));
        double distanceToState = Math.abs(Math.hypot(statePoint.x - startPose.getX(), statePoint.y - startPose.getY()));

        double xOut = robot.pid.xPID.calculate(distanceToState * Math.cos(angle) - (pose.getX() - startPose.getX()));
        double yOut = robot.pid.yPID.calculate(distanceToState * Math.sin(angle) - (pose.getY() - startPose.getY()));
        headingError = AngleUnit.normalizeRadians(desiredPose.getHeading() - pose.getHeading());
        double headingOut = robot.pid.headingPID.calculate(headingError);
        robot.drive.moveTeleOp(yOut, xOut, -headingOut);

        isDone = profile.duration() < profileTime.seconds();
    }
}
