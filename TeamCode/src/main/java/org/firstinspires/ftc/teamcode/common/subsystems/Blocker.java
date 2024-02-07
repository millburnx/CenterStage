package org.firstinspires.ftc.teamcode.common.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.common.utils.SubsystemsHardware;

public class Blocker extends SubsystemBase {
    private SubsystemsHardware subsystems;
    public static boolean isUp = false;
    public double target;

    public BlockerState blockerState = BlockerState.REST;
    public static double rest = 0, release = 0.4;

    public enum BlockerState {
        REST,
        RELEASE
    }

    public Blocker(SubsystemsHardware subsystems) {
        target = 1000;
        this.subsystems = subsystems;
        update(BlockerState.REST);
    }

    public void update(BlockerState state) {
        blockerState = state;
        isUp = state != BlockerState.REST;
        switch (state) {
            case REST:
                target = rest;
                subsystems.blocker.setPosition(rest);
                break;
            case RELEASE:
                target = release;
                subsystems.blocker.setPosition(release);
                break;
        }
    }public void updatePos(int index) {
        switch (index) {
            case 0:
                this.update(BlockerState.REST);
                break;
            case 1:
                this.update(BlockerState.RELEASE);
                break;
        }
    }

    public BlockerState getDepositState() {
        return blockerState;
    }
    public double getTarget(){
        return target;
    }
    public double getPosition(){
        return subsystems.blocker.getPosition();
    }



}
