package org.firstinspires.ftc.teamcode.common.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.common.utils.SubsystemsHardware;

public class Hook extends SubsystemBase {
    private SubsystemsHardware subsystems;
    public double target;
    public int ticks;

    public HookState hookState = HookState.REST;
    public static double rest = 1, hook = 0.95;

    public enum HookState {
        REST,
        HOOK
    }

    public Hook(SubsystemsHardware subsystems) {
        ticks = 0;
        this.subsystems = subsystems;
        update(HookState.REST);
    }

    public HookState update(HookState state) {
        ticks = 0;
        hookState = state;
        switch (state) {
            case REST:
                subsystems.depositHook.setPosition(rest);
                break;
            case HOOK:
                subsystems.depositHook.setPosition(hook);
                break;
        }
        return hookState;
    }


    public HookState getDepositState() {
        return hookState;
    }
    public void loop() {
        ticks+=1;
    }
    public double getTarget() {
        return target;
    }
    public double getPosition() {
        return subsystems.depositHook.getPosition();
    }



}
