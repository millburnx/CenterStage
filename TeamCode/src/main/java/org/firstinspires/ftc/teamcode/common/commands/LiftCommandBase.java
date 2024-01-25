package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.subsystems.Lift;
import org.firstinspires.ftc.teamcode.common.subsystems.Lift;

public class LiftCommandBase extends CommandBase {
    private Lift liftobj;
    private  Lift.LiftStates stateobj;
    public LiftCommandBase(Lift lift, Lift.LiftStates state) {
        liftobj = lift;
        stateobj = state;
        addRequirements(lift);
    }
    @Override
    public void initialize(){
        liftobj.update(stateobj);
    }

    @Override
    public boolean isFinished(){
        return liftobj.isFinished();
    }

}