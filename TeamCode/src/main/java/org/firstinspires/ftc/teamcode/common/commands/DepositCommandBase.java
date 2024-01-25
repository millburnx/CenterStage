package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.common.subsystems.Lift;
import org.firstinspires.ftc.teamcode.common.subsystems.Lift;

public class DepositCommandBase extends CommandBase {
    private Deposit depositobj;
    private  Deposit.DepositState stateobj;
    public DepositCommandBase(Deposit deposit, Deposit.DepositState state) {
        depositobj = deposit;
        stateobj = state;
        addRequirements(deposit);
    }
    @Override
    public void initialize(){
        depositobj.ticks = 0;

        depositobj.update(stateobj);
    }

    @Override
    public boolean isFinished(){
        return depositobj.ticks>100;
    }

}