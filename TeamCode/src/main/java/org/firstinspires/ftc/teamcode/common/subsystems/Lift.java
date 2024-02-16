package org.firstinspires.ftc.teamcode.common.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.utils.SubsystemsHardware;

@Config
public class Lift extends SubsystemBase {
    private SubsystemsHardware subsystems;
    public LiftStates liftStates = LiftStates.DOWN;
    public PIDController controller;
    public static double p = 0.01, i = 0, d = 0.001;
    public static double f = 0.01;
    public static int target = 0;

    public boolean switcher;
    private final double ticks_in_degree = 8192/360.0;


    public static boolean isUp = false;
    public static int rowPos = 0;

    public int ticker;

    public static int DOWN_POS = 0, AUTON_POS = 750, POS1_POS = 1200, POS2_POS = 1750, POS3_POS = 1500, CLIMB_POS = 1700;

    public enum LiftStates {
        DOWN,
        AUTON,
        POS1,
        POS2,
        POS3,
        CLIMB
    }

    public Lift(SubsystemsHardware subsystems) {
        switcher = false;
        this.subsystems = subsystems;
        this.controller = new PIDController(p, i, d);
        rowPos = 0;

        target = getStatePos(LiftStates.DOWN);
        ticker = 0;
    }

    public void update(LiftStates state) {
        isUp = state != LiftStates.DOWN;
        switch (state) {
            case DOWN:
                switcher = true;
                target = DOWN_POS;
                liftStates = state;
                rowPos = 0;
                break;
            case AUTON:
                target = AUTON_POS;
                liftStates = state;
                break;
            case POS1:
                target = POS1_POS;
                liftStates = state;
                rowPos = 1;
                break;
            case POS2:
                target = POS2_POS;
                liftStates = state;
                rowPos = 2;
                break;
            case POS3:
                target = POS3_POS;
                liftStates = state;
                rowPos = 3;
                break;
            case CLIMB:
                target = CLIMB_POS;
                liftStates = state;
        }
    }

    public void updatePos(int index) {
        switch (index) {
            case 0:
                this.update(LiftStates.DOWN);
            case 1:
                this.update(LiftStates.POS1);
                break;
            case 2:
                this.update(LiftStates.POS2);
            case 3:
                this.update(LiftStates.POS3);
        }
    }

    public void changeIndex(int amount) {
        rowPos+=amount;


        if (rowPos<0) {
            rowPos = 0;
        } else if (rowPos >3) {
            rowPos = 3;
        }
        updatePos(rowPos);
    }

    public void loop() {
        controller.setPID(p, i, d);
        int rightPos = subsystems.rightLift.getCurrentPosition();
        double pid = controller.calculate(rightPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
        double power = pid + ff;
        if (switcher && target == DOWN_POS && ticker>200) {
            subsystems.rightLift.setPower(0);
            subsystems.leftLift.setPower(0);
        }
        else {
            subsystems.rightLift.setPower(power);
            subsystems.leftLift.setPower(power);
        }
//        subsystems.rightLift.setPower(power);
//        subsystems.leftLift.setPower(power);
        ticker += 1;

    }


    public boolean isFinished() {
        if (Math.abs(subsystems.rightLift.getCurrentPosition()-target)<100) {
//            if (switcher && target == DOWN_POS) {
//                subsystems.rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                subsystems.leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                subsystems.rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                subsystems.leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                //controller.reset();
//                switcher = false;
//            }
            return true;
        }
        return false;
    }


    public int getStatePos(LiftStates state) {
        switch (state) {
            case DOWN:
                return DOWN_POS;
            case POS1:
                return POS1_POS;
            case POS2:
                return POS2_POS;
            case POS3:
                return POS3_POS;
            case CLIMB:
                return CLIMB_POS;
        }
        return 0;
    }

    public LiftStates getLiftStates() {
        return liftStates;
    }
}
