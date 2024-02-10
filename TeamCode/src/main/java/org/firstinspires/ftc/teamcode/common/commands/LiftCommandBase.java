package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.subsystems.Lift;
import org.firstinspires.ftc.teamcode.common.subsystems.Lift;

public class LiftCommandBase extends CommandBase {
    private Lift liftobj;
    private  Lift.LiftStates stateobj;

    boolean switcher;
    public LiftCommandBase(Lift lift, Lift.LiftStates state, boolean switcherr) {
        liftobj = lift;
        stateobj = state;
        liftobj.switcher=switcherr;
        addRequirements(lift);
    }
    public LiftCommandBase(Lift lift, Lift.LiftStates state) {
        liftobj = lift;
        stateobj = state;
        liftobj.switcher = false;
        addRequirements(lift);
    }
    @Override
    public void initialize(){
        liftobj.update(stateobj);
        liftobj.ticker = 0;
    }

    @Override
    public boolean isFinished(){
        return liftobj.isFinished();
    }

}