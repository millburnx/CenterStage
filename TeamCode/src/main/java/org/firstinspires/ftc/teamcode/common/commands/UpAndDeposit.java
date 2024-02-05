package org.firstinspires.ftc.teamcode.common.commands;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.common.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.common.subsystems.Lift;

public class UpAndDeposit extends SequentialCommandGroup{

    public UpAndDeposit(Lift lift, Deposit deposit, Blocker blocker, int pos, Telemetry t){
        if(pos==0){
            addCommands(
                    new DepositCommandBase(deposit, Deposit.DepositState.INTAKE, t),
                    new LiftCommandBase(lift, Lift.LiftStates.DOWN),
                    new BlockerCommand(blocker, Blocker.BlockerState.REST, t)


            );
        }
        else if(pos ==1){
            addCommands(
                    new LiftCommandBase(lift, Lift.LiftStates.POS1),
            new DepositCommandBase(deposit, Deposit.DepositState.DEPOSIT1, t)
            );

        }
        else if(pos ==2){
            addCommands(
                    new LiftCommandBase(lift, Lift.LiftStates.POS2),
            new DepositCommandBase(deposit, Deposit.DepositState.DEPOSIT2, t)
            );

        }
        else{
            addCommands(
                    new LiftCommandBase(lift, Lift.LiftStates.POS3),
                    new DepositCommandBase(deposit, Deposit.DepositState.DEPOSIT3, t)
            );
        }
        addRequirements(lift, deposit);

    }
}
