package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.subsystems.Intake;

public class IntakeCommand extends InstantCommand {
    public IntakeCommand(Intake intake, Intake.IntakeState state) {
        super (
                () -> intake.update(state)
        );
    }
}
