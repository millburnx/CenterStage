package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.HardwareDevice;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ServoDeposit {
    public ServoEx rightDeposit;
    public ServoEx leftDeposit;

    public ServoDeposit(HardwareMap hardwareMap) {
        rightDeposit = new SimpleServo(
                hardwareMap, "rightDeposit", 0, 120, AngleUnit.DEGREES
        );
        leftDeposit = new SimpleServo(
                hardwareMap, "leftDeposit", 0, 120, AngleUnit.DEGREES
        );
        leftDeposit.setInverted(true);

        rightDeposit.setPosition(0);
        leftDeposit.setPosition(0);
    }
}
