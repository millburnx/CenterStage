package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ctrl.CurvePoint;
import org.firstinspires.ftc.teamcode.ctrl.MoveTo;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.ArrayList;

@Autonomous(name="autoPP", group="Linear Opmode")
public class autonPP extends OpMode {
    MoveTo moveTo;
    Robot robot;
    Telemetry telemetry;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        telemetry.addData("Status", "Initialized");
        moveTo = new MoveTo(robot, telemetry);
    }

    @Override
    public void loop() {
        ArrayList<CurvePoint> allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(0, 0, 1.0, 1.0, 50.0, Math.toRadians(50), 1.0));
        allPoints.add(new CurvePoint(180, 180, 1.0, 1.0, 50.0, Math.toRadians(50), 1.0));
        allPoints.add(new CurvePoint(220, 180, 1.0, 1.0, 50.0, Math.toRadians(50), 1.0));
        allPoints.add(new CurvePoint(280, 50, 1.0, 1.0, 50.0, Math.toRadians(50), 1.0));
        allPoints.add(new CurvePoint(180, 0, 1.0, 1.0, 50.0, Math.toRadians(50), 1.0));

        MoveTo.followCurve(allPoints, Math.toRadians(90));
    }
}
