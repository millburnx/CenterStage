package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.common.subsystems.Lift;
import org.firstinspires.ftc.teamcode.common.subsystems.Lift;

public class DepositCommandBase extends CommandBase {
    private Deposit depositobj;
    public Telemetry telemetry;
    private  Deposit.DepositState stateobj;
    public DepositCommandBase(Deposit deposit, Deposit.DepositState state, Telemetry t) {
        depositobj = deposit;
        stateobj = state;
        addRequirements(deposit);
        telemetry = t;
    }
    @Override
    public void initialize(){
        depositobj.ticks = 0;

        depositobj.update(stateobj);
    }

    @Override
    public boolean isFinished(){
        telemetry.addLine("finished");
        telemetry.update();
        return depositobj.ticks>50;
//        return Math.abs(depositobj.getTarget()-depositobj.getPosition())<0.01;
    }

}