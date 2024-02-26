package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.utils.UltrasonicDistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Ultrasonic extends CommandBase {
    public Telemetry telemetry;
    UltrasonicDistanceSensor ultrasonic;
    public Ultrasonic(UltrasonicDistanceSensor UDS) {
        ultrasonic = UDS;
    }
    @Override
    public void initialize() {

    }

    @Override
    public boolean isFinished() {
        return ultrasonic.getDistance(DistanceUnit.INCH)>100;
    }

}