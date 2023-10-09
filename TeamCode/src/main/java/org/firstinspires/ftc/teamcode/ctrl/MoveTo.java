package org.firstinspires.ftc.teamcode.ctrl;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.opencv.core.Point;

import java.lang.reflect.Array;
import java.util.ArrayList;

public class MoveTo {
    private static Robot robot;
    private FtcDashboard dashboard;
    private Telemetry telemetry;
    public boolean isDone;
    private double currDistance, headingError;
    private final ElapsedTime profileTime = new ElapsedTime();

    public MoveTo(Robot robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public void point(Pose2d pose, Pose2d desiredPose, Pose2d startPose, boolean update) {
        MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(0, 0, 0), new MotionState(1, 0, 0), 2, 3, 4);
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

    // pure pursuit
    public static void followCurve(ArrayList<CurvePoint> allPoints, double followAngle) {
        CurvePoint followMe = getFollowPointPath(allPoints, new Point(robot.odom.getPos().getX(), robot.odom.getPos().getY()), allPoints.get(0).followDistance);
        goToPosition(followMe.x, followMe.y, followMe.moveSpeed, followAngle, followMe.turnSpeed);
    }

    public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Point robotLocation, double followRadius) {
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));
        for(int i = 0; i < pathPoints.size()-1; i++) {
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i+1);

            ArrayList<Point> intersections = MathUtils.lineCircleIntersection(robotLocation, followRadius, startLine.toPoint(), endLine.toPoint());

            double closestAngle = 10000000;

            for(Point thisIntersection : intersections) {
                double angle = Math.atan2(thisIntersection.y - robot.odom.getPos().getY(), thisIntersection.x - robot.odom.getPos().getX());
                double deltaAngle = Math.abs(MathUtils.AngleWrap(angle - robot.odom.getPos().getHeading()));

                if(deltaAngle < closestAngle) {
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }
            }
        }
        return followMe;
    }

    public static void goToPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed) {
        double distanceToTarget = Math.hypot(x-robot.odom.getPos().getX(), y-robot.odom.getPos().getY());
        double absoluteAngleToTarget = Math.atan2(y-robot.odom.getPos().getY(), x-robot.odom.getPos().getX());
        double relativeAngleToPoint = MathUtils.AngleWrap(absoluteAngleToTarget - (robot.odom.getPos().getHeading() - Math.toRadians(90)));
        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;

        robot.drive.moveTeleOp(
                movementXPower*movementSpeed,
                movementYPower*movementSpeed,
                Range.clip(relativeTurnAngle/Math.toRadians(30), -1, 1)*turnSpeed
        );

        if(distanceToTarget < 10) {
            robot.drive.moveTeleOp(
                    movementXPower*movementSpeed,
                    movementYPower*movementSpeed,
                    0
            );
        }
    }
}
