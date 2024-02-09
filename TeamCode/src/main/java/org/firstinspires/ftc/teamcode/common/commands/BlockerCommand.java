package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.common.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.common.subsystems.Lift;
import org.firstinspires.ftc.teamcode.common.subsystems.Lift;

public class BlockerCommand extends CommandBase {
    private Blocker blockerobj;
    public Telemetry telemetry;
    private  Blocker.BlockerState stateobj;
    public BlockerCommand(Blocker blocker, Blocker.BlockerState state, Telemetry t) {
        blockerobj = blocker;
        stateobj = state;
        addRequirements(blocker);
        telemetry = t;
    }
    @Override
    public void initialize(){
        blockerobj.ticks = 0;
        blockerobj.update(stateobj);

    }

    @Override
    public boolean isFinished(){
        telemetry.addLine("finished");
        telemetry.update();

//        return Math.abs(blockerobj.getTarget()-blockerobj.getPosition())<0.01;
        return blockerobj.ticks>80;
    }

}