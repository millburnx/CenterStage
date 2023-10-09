package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;

@Config
public class PID {
    public static PIDController headingPID;
    public static PIDController xPID;
    public static PIDController yPID;

    public static double headingP, xP, yP;

    public PID() {
        headingP = 1;
        xP = 1;
        yP = 1;

        headingPID = new PIDController(headingP, 0, 5, 0, 0.1);
        xPID = new PIDController(xP, 0, 5, 0, 0.1);
        yPID = new PIDController(yP, 0, 5, 0, 0.1);
    }

    public void setPID(double Kp, double Kd, double Ki, double Kf) {
        xPID.setPIDF(Kp, Kd, Ki, Kf);
        yPID.setPIDF(Kp, Kd, Ki, Kf);
    }

    public void setHeadingPID(double Kp, double Kd, double Ki, double Kf) {
        headingPID.setPIDF(Kp, Kd, Ki, Kf);
    }
}
