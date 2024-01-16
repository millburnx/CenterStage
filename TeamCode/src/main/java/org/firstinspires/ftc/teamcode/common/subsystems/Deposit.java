package org.firstinspires.ftc.teamcode.common.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.common.utils.SubsystemsHardware;

public class Deposit extends SubsystemBase {
    private SubsystemsHardware subsystems;
    public DepositState depositState = DepositState.INTAKE;

    public static double intakePos = 0.32, deposit1Pos = 0.7, deposit2Pos = 0.8, deposit3Pos = 0.9;

    public enum DepositState {
        INTAKE,
        DEPOSIT1,
        DEPOSIT2,
        DEPOSIT3
    }

    public Deposit(SubsystemsHardware subsystems) {
        this.subsystems = subsystems;
        update(DepositState.INTAKE);
    }

    public void update(DepositState state) {
        depositState = state;
        switch (state) {
            case INTAKE:
                subsystems.rightDeposit.setPosition(intakePos);
                subsystems.leftDeposit.setPosition(intakePos);
                break;
            case DEPOSIT1:
                subsystems.rightDeposit.setPosition(deposit1Pos);
                subsystems.leftDeposit.setPosition(deposit1Pos);
                break;
            case DEPOSIT2:
                subsystems.rightDeposit.setPosition(deposit2Pos);
                subsystems.leftDeposit.setPosition(deposit2Pos);
            case DEPOSIT3:
                subsystems.rightDeposit.setPosition(deposit3Pos);
                subsystems.leftDeposit.setPosition(deposit3Pos);
        }
    }
}
