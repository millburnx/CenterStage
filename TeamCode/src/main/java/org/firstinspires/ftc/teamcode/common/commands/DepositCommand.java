package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.subsystems.Deposit;

public class DepositCommand extends InstantCommand {
    public DepositCommand(Deposit deposit, Deposit.DepositState state) {
        super(
                () -> deposit.update(state)
        );
    }
}
