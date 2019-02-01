package frc.robot.Utility;

public class PIDController {

    private double error;
    private double lastError;
    
    private double integral;
    private double kP;
    private double kI;
    private double kD;
    private double kF;
    
    private final double DT = 0.02; // In seconds so 20ms
    
    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = 0;
    
        this.lastError = 0.0;
        this.error = 0.0;
        this.integral = 0.0;
    }

    public PIDController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    
        this.lastError = 0.0;
        this.error = 0.0;
        this.integral = 0.0;
    }
    
    public double updatePID(double position, double goal) {
        error = goal - position;
        integral += error * DT;
        double output = (kP * error) + (kI * integral) + (kD * (error - lastError) / DT) + kF;

        //Keeps output between -1 and 1 for motor control
        if(output >= 1)
            output = 1;
        else if(output <= -1)
            output = -1;
        
        lastError = error;
        return output;
    }
    
    public void resetController() {
        this.lastError = 0.0;
        this.error = 0.0;
        this.integral = 0.0;
    }
    
    public void updateConstants(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }
    
    public double getError() {
        return lastError;
    }
}










