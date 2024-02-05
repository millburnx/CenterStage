package org.firstinspires.ftc.teamcode.common.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.common.utils.SubsystemsHardware;

public class Blocker extends SubsystemBase {
    private SubsystemsHardware subsystems;
    public static boolean isUp = false;
    public int ticks;

    public BlockerState blockerState = BlockerState.REST;
    public static double rest = 0, release = 0.4;

    public enum BlockerState {
        REST,
        RELEASE
    }

    public Blocker(SubsystemsHardware subsystems) {

        this.subsystems = subsystems;
        update(BlockerState.REST);
    }

    public void update(BlockerState state) {
        ticks = 0;
        blockerState = state;
        isUp = state != BlockerState.REST;
        switch (state) {
            case REST:
                subsystems.blocker.setPosition(rest);
                break;
            case RELEASE:
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
    public void loop(){
        ticks+=1;
    }
}
