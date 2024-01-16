package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.subsystems.Lift;
import org.firstinspires.ftc.teamcode.common.subsystems.Lift;

public class LiftCommand extends InstantCommand {
    public LiftCommand(Lift lift, Lift.LiftStates state) {
        super(
                () -> lift.update(state)
        );
    }
}