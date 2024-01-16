package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.subsystems.Lift;

public class IncrementLiftCommand extends InstantCommand {
    public IncrementLiftCommand(Lift lift, int amount) {
        super(
                () -> lift.updatePos(amount)
        );
    }
}
