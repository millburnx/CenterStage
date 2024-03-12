package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.subsystems.Intake;

public class IntakeUpCommand extends CommandBase {
    Intake intakeObj;
    double poss;
    public IntakeUpCommand(Intake intake, double pos) {
        intakeObj = intake;
        poss = pos;
    }
    @Override
    public void initialize(){
        intakeObj.updatePosition(poss);
        intakeObj.ticks = 0;
    }

    @Override
    public boolean isFinished() {
        return intakeObj.ticks>20;
    }
}
