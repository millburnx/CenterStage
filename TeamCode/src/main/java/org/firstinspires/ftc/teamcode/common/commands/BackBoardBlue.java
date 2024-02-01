package org.firstinspires.ftc.teamcode.common.commands;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.common.subsystems.Intake;
import org.firstinspires.ftc.teamcode.common.subsystems.Lift;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;




public class BackBoardBlue extends SequentialCommandGroup {
    public BackBoardBlue(Lift lift, Deposit deposit, Intake intake, SampleMecanumDrive robot, int position, Telemetry telemetry){

        addCommands(
                new AutonSeqBackBoardBlue1(robot, position, telemetry),
                new IntakeCommand(intake, Intake.IntakeState.AUTON_OUT),
                new AutonSeqBackBoardBlue1_5(robot, position, telemetry),
                new UpAndDeposit(lift, deposit, 2,telemetry),
                new AutonSeqBackBoardBlue2(robot, position),
                new UpAndDeposit(lift, deposit, 0, telemetry)
        );


        addRequirements(lift, deposit);
    }
}
