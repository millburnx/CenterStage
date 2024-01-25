package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commands.UpAndDeposit;
import org.firstinspires.ftc.teamcode.common.commands.DepositCommandBase;
import org.firstinspires.ftc.teamcode.common.drive.Drive;
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

        subsystems.enabled = true;
        subsystems.drone.setPosition(Math.toRadians(90));
        intake.update(Intake.IntakeState.IN);
        deposit.update(Deposit.DepositState.INTAKE);
        lift.update(Lift.LiftStates.DOWN);
    }

    @Override
    public void run() {
        super.run();

        if (gamepad1.a) {
            schedule(new UpAndDeposit(lift, deposit, 0));
        }
        else if (gamepad1.b) {
            schedule(new UpAndDeposit(lift, deposit, 1));
        }

        else if (gamepad1.x) {
            schedule(new UpAndDeposit(lift, deposit, 2));

        }
        else if (gamepad1.y) {
            schedule(new UpAndDeposit(lift, deposit, 3));

        }
        lift.loop();
        deposit.loop();

        double power = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        drive.moveTeleOp(power, strafe, turn);

        if(gamepad1.right_trigger>0.3){
            lift.target +=10;
        }
        if(gamepad1.left_trigger>0.3){
            lift.target -=10;
        }

        if (gamepad1.dpad_down) {
            schedule(new DepositCommandBase(deposit, Deposit.DepositState.INTAKE));
        } else if (gamepad1.dpad_up) {
            schedule(new DepositCommandBase(deposit, Deposit.DepositState.DEPOSIT1));
        } else if (gamepad1.dpad_right) {
            schedule(new DepositCommandBase(deposit, Deposit.DepositState.DEPOSIT2));
        } else if (gamepad1.dpad_left) {
            schedule(new DepositCommandBase(deposit, Deposit.DepositState.DEPOSIT3));
        }

        telemetry.addData("Lift State: ", lift.getLiftStates());
        telemetry.addData("Lift pos: ", lift.rowPos);

        telemetry.addData("Deposit State: ", deposit.getDepositState());
        telemetry.addData("Deposit pos: ", deposit.rowPos);

        telemetry.addData("target: ", Lift.target);
        telemetry.addData("Intake state: ", deposit.getDepositState());

        telemetry.addData("deposit ticks", deposit.ticks);
        telemetry.update();
    }
}