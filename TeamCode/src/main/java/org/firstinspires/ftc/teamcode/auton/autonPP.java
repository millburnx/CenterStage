package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ctrl.CurvePoint;
import org.firstinspires.ftc.teamcode.ctrl.PurePursuit;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.ArrayList;

@Autonomous(name="autoPP", group="Linear Opmode")
public class autonPP extends OpMode {
    PurePursuit purePursuit;
    Robot robot;
    Telemetry telemetry;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, gamepad1);
        telemetry.addData("Status", "Initialized");
        purePursuit = new PurePursuit(robot, telemetry);
    }

    @Override
    public void loop() {
        ArrayList<CurvePoint> allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(0, 0, 1.0, 1.0, 50.0, Math.toRadians(50), 1.0));
        allPoints.add(new CurvePoint(180, 180, 1.0, 1.0, 50.0, Math.toRadians(50), 1.0));
        allPoints.add(new CurvePoint(220, 180, 1.0, 1.0, 50.0, Math.toRadians(50), 1.0));
        allPoints.add(new CurvePoint(280, 50, 1.0, 1.0, 50.0, Math.toRadians(50), 1.0));
        allPoints.add(new CurvePoint(180, 0, 1.0, 1.0, 50.0, Math.toRadians(50), 1.0));

        PurePursuit.followCurve(allPoints, Math.toRadians(90));
    }
}
