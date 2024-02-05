package org.firstinspires.ftc.teamcode.common.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class DashTelemetry {
    public static void drawRobot(Canvas canvas, Pose2d pose) {
        canvas.strokeCircle(pose.getX(), pose.getY(), 9);
        Vector2d v = pose.headingVec().times(9);
        double x1 = pose.getX() + v.getX() / 2, y1 = pose.getY() + v.getY() / 2;
        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);
    }

    public void drawField(Pose2d pose, FtcDashboard dash){
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        fieldOverlay.setStrokeWidth(1);
        fieldOverlay.setStroke("#3F51B5");
        drawRobot(fieldOverlay, pose);
//        fieldOverlay.setStroke("#51B53F");
//        drawRobot(fieldOverlay,desiredPose);
//        fieldOverlay.setStroke("#B53F51");
//        drawRobot(fieldOverlay,startPose);

        packet.put("x: ", pose.getX());
        packet.put("y: ", pose.getY());
        packet.put("heading (degrees):", Math.toDegrees(pose.getHeading()));

        dash.sendTelemetryPacket(packet);
    }

    public void drawFieldRed(Pose2d pose, FtcDashboard dash){
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        fieldOverlay.setStrokeWidth(1);
        fieldOverlay.setStroke("#51B53F");
        drawRobot(fieldOverlay, pose);
//        fieldOverlay.setStroke("#51B53F");
//        drawRobot(fieldOverlay,desiredPose);
//        fieldOverlay.setStroke("#B53F51");
//        drawRobot(fieldOverlay,startPose);

        packet.put("x: ", pose.getX());
        packet.put("y: ", pose.getY());
        packet.put("heading (degrees):", Math.toDegrees(pose.getHeading()));

        dash.sendTelemetryPacket(packet);
    }

    public void drawFieldAuto(Pose2d pose, Pose2d desiredPose, Pose2d startPose, FtcDashboard dash){
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        fieldOverlay.setStrokeWidth(1);
        fieldOverlay.setStroke("#3F51B5");
        drawRobot(fieldOverlay, pose);
        fieldOverlay.setStroke("#51B53F");
        drawRobot(fieldOverlay,desiredPose);
        fieldOverlay.setStroke("#B53F51");
        drawRobot(fieldOverlay,startPose);

        packet.put("x: ", pose.getX());
        packet.put("y: ", pose.getY());
        packet.put("heading (degrees):", Math.toDegrees(pose.getHeading()));

        dash.sendTelemetryPacket(packet);
    }
}