package org.firstinspires.ftc.teamcode.ctrl;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.opencv.core.Point;

import java.util.ArrayList;

public class PurePursuit {
    private static Robot robot;
    private FtcDashboard dashboard;
    private Telemetry telemetry;
    public boolean isDone;
    private double currDistance, headingError;
    private final ElapsedTime profileTime = new ElapsedTime();

    public PurePursuit(Robot robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    // pure pursuit
    public static void followCurve(ArrayList<CurvePoint> allPoints, double followAngle) {
        CurvePoint followMe = getFollowPointPath(allPoints, new Point(robot.drive.getPos().getX(), robot.drive.getPos().getY()), allPoints.get(0).followDistance);
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
                double angle = Math.atan2(thisIntersection.y - robot.drive.getPos().getY(), thisIntersection.x - robot.drive.getPos().getX());
                double deltaAngle = Math.abs(MathUtils.AngleWrap(angle - robot.drive.getPos().getHeading()));

                if(deltaAngle < closestAngle) {
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }
            }
        }
        return followMe;
    }

    public static void goToPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed) {
        double distanceToTarget = Math.hypot(x-robot.drive.getPos().getX(), y-robot.drive.getPos().getY());
        double absoluteAngleToTarget = Math.atan2(y-robot.drive.getPos().getY(), x-robot.drive.getPos().getX());
        double relativeAngleToPoint = MathUtils.AngleWrap(absoluteAngleToTarget - (robot.drive.getPos().getHeading() - Math.toRadians(90)));
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
