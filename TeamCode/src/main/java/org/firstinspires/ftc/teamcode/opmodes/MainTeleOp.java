package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commands.BlockerCommand;
import org.firstinspires.ftc.teamcode.common.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.UpAndDeposit;
import org.firstinspires.ftc.teamcode.common.commands.DepositCommandBase;
import org.firstinspires.ftc.teamcode.common.drive.Drive;
import org.firstinspires.ftc.teamcode.common.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.common.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.common.subsystems.Intake;
import org.firstinspires.ftc.teamcode.common.subsystems.Lift;
import org.firstinspires.ftc.teamcode.common.utils.SubsystemsHardware;

@Config
@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends CommandOpMode {
    private final SubsystemsHardware subsystems = SubsystemsHardware.getInstance();
    private Drive drive;
    private Intake intake;
    private Lift lift;
    private Deposit deposit;
    private Blocker blocker;

    GamepadEx gamepadEx;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        subsystems.init(hardwareMap);
        drive = new Drive(hardwareMap);
        intake = new Intake(subsystems);
        lift = new Lift(subsystems);
        deposit = new Deposit(subsystems);
        blocker = new Blocker(subsystems);


        subsystems.enabled = true;
        subsystems.drone.setPosition(Math.toRadians(90));
//        intake.update(Intake.IntakeState.IN);
        deposit.update(Deposit.DepositState.INTAKE);
        lift.update(Lift.LiftStates.DOWN);
    }

    @Override
    public void run() {
        super.run();

        if (gamepad1.triangle) {
            blocker.target = 0;
            schedule(new UpAndDeposit(lift, deposit, blocker, 0, telemetry));
        }
        else if (gamepad1.circle) {
            schedule(new UpAndDeposit(lift, deposit,blocker, 1, telemetry));
        }

        else if (gamepad1.square) {
            schedule(new UpAndDeposit(lift, deposit,blocker, 2, telemetry));

        }


//        else if (gamepad1.square) {
//            schedule(new UpAndDeposit(lift, deposit, 3, telemetry));
//
//        }
        lift.loop();
        deposit.loop();
        blocker.loop();

        double power = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        drive.moveTeleOp(power, strafe, turn);

        if(subsystems.rightLift.getCurrentPosition() <1800 && gamepad1.right_trigger>0.3){
            lift.target += 5;

        }
        else if(gamepad1.left_trigger>0.3){
            lift.target -= 5;
        }

        if (gamepad1.dpad_down) {
            schedule(new DepositCommandBase(deposit, Deposit.DepositState.INTAKE, telemetry));
        } else if (gamepad1.dpad_up) {
            schedule(new DepositCommandBase(deposit, Deposit.DepositState.DEPOSIT1, telemetry));
        }
        if (gamepad2.right_bumper) {
            schedule(new IntakeCommand(intake, Intake.IntakeState.IN));
        }
        else if(gamepad2.left_bumper){
            schedule(new IntakeCommand(intake, Intake.IntakeState.OFF));

        }

        if(gamepad1.right_stick_button){
            telemetry.addLine("0.5");
            subsystems.intakeRight.setPosition(0.10);
            subsystems.intakeLeft.setPosition(0.10);

        }
        else if(gamepad1.left_stick_button){
            telemetry.addLine("0");
            subsystems.intakeLeft.setPosition(0);
            subsystems.intakeRight.setPosition(0);
        }

        if(gamepad1.right_bumper){
            telemetry.addLine("rest");
            schedule(new BlockerCommand(blocker, Blocker.BlockerState.REST, telemetry));
        }
        else if(gamepad1.left_bumper){
            telemetry.addLine("release");
            schedule(new BlockerCommand(blocker, Blocker.BlockerState.RELEASE, telemetry));
        }

        telemetry.addData("Lift State: ", lift.getLiftStates());
        telemetry.addData("Lift pos: ", lift.rowPos);

        telemetry.addData("Deposit State: ", deposit.getDepositState());
        telemetry.addData("Deposit pos: ", subsystems.rightLift.getCurrentPosition());

        telemetry.addData("target: ", Lift.target);
        telemetry.addData("Intake state: ", deposit.getDepositState());

        telemetry.addData("deposit ticks", deposit.ticks);
        telemetry.addData("deposit encoder", deposit.getPosition());

        telemetry.addData("blocker ticks", blocker.ticks);
        telemetry.addData("cond",  blocker.ticks>150);

        telemetry.update();
    }
}