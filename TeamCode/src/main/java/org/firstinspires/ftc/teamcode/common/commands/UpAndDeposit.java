package org.firstinspires.ftc.teamcode.common.commands;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.common.subsystems.Lift;

public class UpAndDeposit extends SequentialCommandGroup{

    public UpAndDeposit(Lift lift, Deposit deposit, int pos){
        if(pos==0){
            addCommands(
                    new DepositCommandBase(deposit, Deposit.DepositState.INTAKE),
                    new LiftCommandBase(lift, Lift.LiftStates.DOWN)


            );
        }
        else if(pos ==1){
            addCommands(
                    new LiftCommandBase(lift, Lift.LiftStates.POS1),
            new DepositCommandBase(deposit, Deposit.DepositState.DEPOSIT1)
            );

        }
        else if(pos ==2){
            addCommands(
                    new LiftCommandBase(lift, Lift.LiftStates.POS2),
            new DepositCommandBase(deposit, Deposit.DepositState.DEPOSIT2)
            );

        }
        else{
            addCommands(
                    new LiftCommandBase(lift, Lift.LiftStates.POS3),
                    new DepositCommandBase(deposit, Deposit.DepositState.DEPOSIT3)
            );
        }
        addRequirements(lift, deposit);

    }
}
