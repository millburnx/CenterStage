package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;

public class PID {
    public PIDController headingPID;
    public PIDController xPID;
    public PIDController yPID;

    public PID() {
        headingPID = new PIDController(0.5, 0, 0, 0, 0.1);
        xPID = new PIDController(0.5, 0, 0, 0, 0.1);
        yPID = new PIDController(0.5, 0, 0, 0, 0.1);
    }

    public void setPID(double Kp, double Kd, double Ki, double Kf) {
        xPID.setPIDF(Kp, Kd, Ki, Kf);
        yPID.setPIDF(Kp, Kd, Ki, Kf);
    }

    public void setHeadingPID(double Kp, double Kd, double Ki, double Kf) {
        headingPID.setPIDF(Kp, Kd, Ki, Kf);
    }
}
