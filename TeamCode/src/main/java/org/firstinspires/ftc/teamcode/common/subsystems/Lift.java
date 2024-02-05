package org.firstinspires.ftc.teamcode.common.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.common.utils.SubsystemsHardware;

@Config
public class Lift extends SubsystemBase {
    private SubsystemsHardware subsystems;
    public LiftStates liftStates = LiftStates.DOWN;
    public PIDController controller;
    public static double p = 0.01, i = 0, d = 0.001;
    public static double f = 0.001;
    public static int target = 0;
    private final double ticks_in_degree = 8192/360.0;

    public static boolean isUp = false;
    public static int rowPos = 0;

    public static int DOWN_POS = 0, POS1_POS = 1200, POS2_POS = 1700, POS3_POS = 1500, CLIMB_POS = 1700;

    public enum LiftStates {
        DOWN,
        POS1,
        POS2,
        POS3,
        CLIMB
    }

    public Lift(SubsystemsHardware subsystems) {
        this.subsystems = subsystems;
        this.controller = new PIDController(p, i, d);
        rowPos = 0;

        target = getStatePos(LiftStates.DOWN);
    }

    public void update(LiftStates state) {
        isUp = state != LiftStates.DOWN;
        switch (state) {
            case DOWN:
                target = DOWN_POS;
                liftStates = state;
                rowPos = 0;
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

        subsystems.rightLift.setPower(power);
        subsystems.leftLift.setPower(power);
    }


    public boolean isFinished() {
        if(Math.abs(subsystems.rightLift.getCurrentPosition()-target)<100){
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
