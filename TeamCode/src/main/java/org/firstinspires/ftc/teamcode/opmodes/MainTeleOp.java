package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.commands.BlockerCommand;
import org.firstinspires.ftc.teamcode.common.commands.HookCommand;
import org.firstinspires.ftc.teamcode.common.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.LiftCommandBase;
import org.firstinspires.ftc.teamcode.common.commands.UpAndDeposit;
import org.firstinspires.ftc.teamcode.common.commands.DepositCommandBase;
import org.firstinspires.ftc.teamcode.common.drive.Drive;
import org.firstinspires.ftc.teamcode.common.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.common.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.common.subsystems.Hook;
import org.firstinspires.ftc.teamcode.common.subsystems.Intake;
import org.firstinspires.ftc.teamcode.common.subsystems.Lift;
import org.firstinspires.ftc.teamcode.common.utils.SubsystemsHardware;

import java.util.Locale;

@Config
@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends CommandOpMode {
    private final SubsystemsHardware subsystems = SubsystemsHardware.getInstance();
    private Drive drive;
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    private Intake intake;
    private Hook hook;
    private Lift lift;
    private Deposit deposit;
    private Blocker blocker;

    GamepadEx gamepadEx;
    boolean lastLeftBumper = false;

    boolean hookToggle;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        subsystems.init(hardwareMap);
        drive = new Drive(hardwareMap);
        hook = new Hook(subsystems);
        intake = new Intake(subsystems);
        lift = new Lift(subsystems);
        deposit = new Deposit(subsystems);
        blocker = new Blocker(subsystems);
        hookToggle = false;

        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");


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
            schedule(new UpAndDeposit(lift, deposit, blocker, hook, 0, telemetry));
        }
        else if (gamepad1.circle) {
            schedule(new UpAndDeposit(lift, deposit,blocker,hook, 1, telemetry));
        }

        else if (gamepad1.square) {
            schedule(new UpAndDeposit(lift, deposit,blocker,hook, 2, telemetry));

        }


        lift.loop();
        deposit.loop();
        hook.loop();
        blocker.loop();


        double power = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        if (gamepad1.dpad_left) {
            strafe = 0.5;
            power = 0;
            turn = 0;
        }
        else if (gamepad1.dpad_right) {
            strafe = -0.5;
            power = 0;
            turn = 0;

        }
        telemetry.addData("Distance (cm)",
                String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
        if(sensorDistance.getDistance(DistanceUnit.INCH)<0.5){
            gamepad1.rumble(500);
        }

        if (deposit.getDepositState() == Deposit.DepositState.DEPOSIT1 || deposit.getDepositState() == Deposit.DepositState.DEPOSIT2 || deposit.getDepositState() == Deposit.DepositState.DEPOSIT3) {
            if (Math.abs(power) > Math.abs(strafe)) {
                drive.moveTeleOp(power, 0, turn);
            } else {
                drive.moveTeleOp(0, strafe, turn);
            }
        } else {
            drive.moveTeleOp(power, strafe, turn);
        }

        if (subsystems.rightLift.getCurrentPosition() <1800 && gamepad1.right_trigger>0.8) {
            lift.target += 10;

        }
        else if (gamepad1.left_trigger>0.8) {
            lift.target -= 10;
        }

        if (gamepad1.dpad_down) {
            schedule(new DepositCommandBase(deposit, Deposit.DepositState.INTAKE, telemetry));
        } else if (gamepad1.dpad_up) {
            schedule(new DepositCommandBase(deposit, Deposit.DepositState.DEPOSIT1, telemetry));
        }
        if (gamepad2.right_bumper) {
            if(hookToggle){
                schedule(new HookCommand(hook, Hook.HookState.REST, telemetry));
                hookToggle = false;
            }
            else {
                schedule(new IntakeCommand(intake, Intake.IntakeState.IN));
                hookToggle = true;
            }
        }
        else if (gamepad2.left_bumper) {
            schedule(new IntakeCommand(intake, Intake.IntakeState.OFF));

        }

        if (gamepad2.right_stick_button){
            schedule(new HookCommand(hook, Hook.HookState.REST, telemetry));
        }
        else if(gamepad2.left_stick_button){
            schedule(new HookCommand(hook, Hook.HookState.HOOK, telemetry));
        }

        if (gamepad2.b) {
            schedule(new LiftCommandBase(lift, Lift.LiftStates.DOWN, true));
        }

        if (gamepad1.right_stick_button) {
            subsystems.intakeRight.setPosition(0.17);
            subsystems.intakeLeft.setPosition(0.17);

        }
        else if (gamepad1.left_stick_button) {
            subsystems.intakeLeft.setPosition(0.085);
            subsystems.intakeRight.setPosition(0.085);
        }

        if(gamepad2.dpad_up){
            subsystems.intakeRight.setPosition(0.17);
            subsystems.intakeLeft.setPosition(0.17);
        }
        else if(gamepad2.dpad_right){
            subsystems.intakeRight.setPosition(0.15);
            subsystems.intakeLeft.setPosition(0.15);
        }
        else if(gamepad2.dpad_down){
            subsystems.intakeRight.setPosition(0.15);
            subsystems.intakeLeft.setPosition(0.15);
        }
        else if(gamepad2.dpad_left){
            subsystems.intakeRight.setPosition(0.11);
            subsystems.intakeLeft.setPosition(0.11);
        }

        if (gamepad1.right_bumper) {
            schedule(new BlockerCommand(blocker, Blocker.BlockerState.REST, telemetry));
        }
        else if (gamepad1.left_bumper && !lastLeftBumper) {
            if (blocker.blockerState.equals(Blocker.BlockerState.REST)) {
                schedule(new BlockerCommand(blocker, Blocker.BlockerState.RELEASE, telemetry));
            } else if (hook.hookState.equals(Hook.HookState.HOOK)) {
                schedule(new HookCommand(hook, Hook.HookState.REST, telemetry));
            }
        }
        lastLeftBumper = gamepad1.left_bumper;

        if (gamepad2.y) {
            subsystems.drone.setPosition(Math.toRadians(30));
        }

        if(gamepad1.cross){
            schedule(new IntakeCommand(intake, Intake.IntakeState.OUT));
        }


        telemetry.addData("Lift target: ", lift.target);
        telemetry.addData("Lift pos: ", subsystems.rightLift.getCurrentPosition());
        telemetry.addData("Lift ticks: ", lift.ticker);


        telemetry.addData("blocker ticks", blocker.ticks);
        telemetry.addData("cond",  blocker.ticks>150);

        telemetry.addData("color: ", sensorDistance.getDistance(DistanceUnit.INCH));

        telemetry.update();
    }
}