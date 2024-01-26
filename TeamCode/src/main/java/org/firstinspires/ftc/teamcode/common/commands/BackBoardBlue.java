package org.firstinspires.ftc.teamcode.common.commands;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.common.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.common.subsystems.Intake;
import org.firstinspires.ftc.teamcode.common.subsystems.Lift;



public class BackBoardBlue extends SequentialCommandGroup {
    public BackBoardBlue(Lift lift, Deposit deposit, Intake intake, SampleMecanumDrive robot, int position){

        addCommands(
                new AutonSeqBackBoardBlue1(robot, position),
                new IntakeCommand(intake, Intake.IntakeState.AUTON_OUT),
                new AutonSeqBackBoardBlue1_5(robot, position),
                new UpAndDeposit(lift, deposit, 2),
                new AutonSeqBackBoardBlue2(robot, position),
                new UpAndDeposit(lift, deposit, 0)
        );
    }
}
