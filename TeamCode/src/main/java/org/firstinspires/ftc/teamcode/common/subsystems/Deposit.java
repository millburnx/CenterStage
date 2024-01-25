package org.firstinspires.ftc.teamcode.common.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.common.utils.SubsystemsHardware;

public class Deposit extends SubsystemBase {
    private SubsystemsHardware subsystems;
    public static int rowPos = 0;
    public static boolean isUp = false;

    public DepositState depositState = DepositState.INTAKE;
    public int ticks = 0;
    public static double intakePos = 0.32, deposit1Pos = 0.7, deposit2Pos = 0.8, deposit3Pos = 0.9;

    public enum DepositState {
        INTAKE,
        DEPOSIT1,
        DEPOSIT2,
        DEPOSIT3
    }

    public Deposit(SubsystemsHardware subsystems) {
        ticks = 0;
        this.subsystems = subsystems;
        update(DepositState.INTAKE);
        rowPos = 0;
    }

    public void update(DepositState state) {
        depositState = state;
        isUp = state != DepositState.INTAKE;
        switch (state) {
            case INTAKE:
                subsystems.rightDeposit.setPosition(intakePos);
                subsystems.leftDeposit.setPosition(intakePos);
                rowPos = 0;
                break;
            case DEPOSIT1:
                subsystems.rightDeposit.setPosition(deposit1Pos);
                subsystems.leftDeposit.setPosition(deposit1Pos);
                rowPos = 1;
                break;
            case DEPOSIT2:
                subsystems.rightDeposit.setPosition(deposit2Pos);
                subsystems.leftDeposit.setPosition(deposit2Pos);
                rowPos = 2;
            case DEPOSIT3:
                subsystems.rightDeposit.setPosition(deposit3Pos);
                subsystems.leftDeposit.setPosition(deposit3Pos);
                rowPos = 3;
        }
    }public void updatePos(int index) {
        switch (index) {
            case 0:
                this.update(DepositState.INTAKE);
                break;
            case 1:
                this.update(DepositState.DEPOSIT1);
                break;
            case 2:
                this.update(DepositState.DEPOSIT2);
            case 3:
                this.update(DepositState.DEPOSIT3);
        }
    }

    public void changeIndex(int amount) {
            rowPos += amount;
            if (rowPos<0) {
                rowPos = 0;
            }
            else if (rowPos>3) {
                rowPos = 3;
            }

            updatePos(rowPos);
        }

    public DepositState getDepositState() {
        return depositState;
    }
    public void loop(){
        ticks+=1;
    }
}
