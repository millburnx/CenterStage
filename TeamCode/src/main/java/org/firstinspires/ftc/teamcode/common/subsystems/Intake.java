package org.firstinspires.ftc.teamcode.common.subsystems;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.common.utils.SubsystemsHardware;

@Config
public class Intake {
    private SubsystemsHardware subsystems;
    public IntakeState intakeState = IntakeState.IN;

    public static double intakeMaxSpeed = 1;
    //0.85
    public static double intakeInSpeed = 0.85;
    public static double intakeInSpeedAlt = 0.45;

    public static double intakeOutSpeed = -0.6;
    public static double intakeOutSpeedAuton = -0.6;
    public static double secondMaxSpeed = -1;
    public static double secondInSpeed = -0.6;
    public static double secondOutSpeed = 0.6;
    public int ticks;

    public enum IntakeState {
        OFF,
        IN,
        INAlt,
        MAX,
        OUT,
        AUTON_OUT
    }

    public Intake(SubsystemsHardware subsystems) {
        this.subsystems = subsystems;
    }

    public void update(IntakeState state) {
        intakeState = state;
        switch (state) {
            case OFF:
                subsystems.stageOne.set(0);
                //subsystems.stageTwo.set(0);
                break;
            case IN:
                subsystems.stageOne.set(intakeInSpeed);
                //subsystems.stageTwo.set(secondInSpeed);
                break;
            case MAX:
                subsystems.stageOne.set(intakeMaxSpeed);
                //subsystems.stageTwo.set(secondMaxSpeed);
                break;
            case OUT:
                subsystems.stageOne.set(intakeOutSpeed);
                //subsystems.stageTwo.set(secondOutSpeed);
                break;
            case AUTON_OUT:
                subsystems.stageOne.set(intakeOutSpeedAuton);
                //subsystems.stageTwo.set(0);
            case INAlt:
                subsystems.stageOne.set(intakeInSpeedAlt);

        }
    }
    public void  updatePosition(double pos) {
        subsystems.intakeRight.setPosition(pos);
        subsystems.intakeLeft.setPosition(pos);
    }
    public void loop(){
        ticks +=1;
    }
}
