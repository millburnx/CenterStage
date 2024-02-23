package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import  org.firstinspires.ftc.teamcode.PixelDetector;


public class PixelOpMode extends CommandOpMode {
    PixelDetector detector;
    @Override
    public void initialize() {
        detector = new PixelDetector(hardwareMap, telemetry);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void run() {
        super.run();
        telemetry.addLine(Integer.toString(detector.getX()));
        telemetry.update();

    }

}
