package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class PixelOpMode extends OpMode {
    PixelDetector detector;

    @Override
    public void init() {
        detector = new PixelDetector(hardwareMap, telemetry);
    }

    @Override
    public void loop(){
        telemetry.addLine(Integer.toString(detector.getX()));
        telemetry.update();
    }
}
