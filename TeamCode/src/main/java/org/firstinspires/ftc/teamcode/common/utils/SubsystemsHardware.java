package org.firstinspires.ftc.teamcode.common.utils;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class SubsystemsHardware {
//    intake
    public MotorEx stageOne;
    //public MotorEx stageTwo;
    public ServoEx intakeLeft;
    public ServoEx intakeRight;

    public ServoEx depositHook;


    //    lift
    public DcMotorEx leftLift;
    public DcMotorEx rightLift;

//    deposit
    public ServoEx rightDeposit;
    public ServoEx leftDeposit;

//    drone
    public ServoEx drone;

    public ServoEx blocker;

    private static SubsystemsHardware instance = null;
    public boolean enabled;
    private HardwareMap hardwareMap;

    public static SubsystemsHardware getInstance() {
        if (instance == null) {
            instance = new SubsystemsHardware();
        }
        instance.enabled = true;
        return instance;
    }

    public void init(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        stageOne = new MotorEx(hardwareMap, "stageOne", Motor.GoBILDA.RPM_1620);
        stageOne.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        stageOne.set(0);
        stageOne.motorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        stageTwo = new MotorEx(hardwareMap, "stageTwo", Motor.GoBILDA.RPM_435);
//        stageTwo.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
//        stageTwo.set(0);
//        stageTwo.motorEx.setMode((DcMotor.RunMode.RUN_WITHOUT_ENCODER));

        intakeLeft = new SimpleServo(
                hardwareMap, "intakeLeft", 0, 180, AngleUnit.DEGREES
        );
        intakeRight = new SimpleServo(
                hardwareMap, "intakeRight", 0, 180, AngleUnit.DEGREES
        );
        intakeLeft.setInverted(true);
        intakeLeft.setPosition(0.067);
        intakeRight.setPosition(0.067);

        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setDirection(DcMotor.Direction.REVERSE);

        rightDeposit = new SimpleServo(
                hardwareMap, "rightDeposit", 0, 120, AngleUnit.DEGREES
        );
        leftDeposit = new SimpleServo(
                hardwareMap, "leftDeposit", 0, 120, AngleUnit.DEGREES
        );
        rightDeposit.setInverted(true);
        rightDeposit.setPosition(0);
        leftDeposit.setPosition(0);

        drone = new SimpleServo(
                hardwareMap, "drone", -360, 360, AngleUnit.DEGREES
        );

        blocker = new SimpleServo(
                hardwareMap, "blocker", 0, 360, AngleUnit.DEGREES
        );
        blocker.setPosition(0);
        depositHook = new SimpleServo(
                hardwareMap, "depositHook", 0, 360, AngleUnit.DEGREES
        );
        depositHook.setPosition(1);
    }
}
