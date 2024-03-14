package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.common.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.common.subsystems.Hook;
import org.firstinspires.ftc.teamcode.common.subsystems.Lift;
import org.firstinspires.ftc.teamcode.common.subsystems.Lift;

public class HookCommand extends CommandBase {
    private Hook hookobj;
    public Telemetry telemetry;
    private Hook.HookState stateobj;
    public HookCommand(Hook hook, Hook.HookState state, Telemetry t) {
        hookobj = hook;
        stateobj = state;
        addRequirements(hook);
        telemetry = t;
    }
    @Override
    public void initialize() {

        hookobj.ticks = 0;
        hookobj.update(stateobj);

    }

    @Override
    public boolean isFinished() {
        return hookobj.ticks>50;
    }

}