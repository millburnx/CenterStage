package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public Drive drive;
    public Odometry odom;

    public Robot(HardwareMap hardwareMap) {
        drive = new Drive(hardwareMap);
        odom = new Odometry(drive);
    }
}
