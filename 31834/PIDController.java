package org.firstinspires.ftc.teamcode;

public class PIDController {
    private double kP, kI, kD;
    private double integral, lastError;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double update(double target, double current) {
        double error = target - current;
        integral += error;
        double derivative = error - lastError;
        lastError = error;
        return kP * error + kI * integral + kD * derivative;
    }

    public void reset() {
        integral = 0;
        lastError = 0;
    }
}
