package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.subsystems.Intake;

public class IntakeUpCommand extends InstantCommand {
    public IntakeUpCommand(Intake intake, int pos) {
        super (
                () -> intake.updatePosition(pos)
        );
    }
}
