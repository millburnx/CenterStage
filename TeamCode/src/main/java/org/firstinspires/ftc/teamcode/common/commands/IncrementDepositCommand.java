package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.subsystems.Deposit;

public class IncrementDepositCommand extends InstantCommand {
    public IncrementDepositCommand(Deposit deposit, int amount) {
        super(
                () -> deposit.changeIndex(amount)
        );
    }
}
