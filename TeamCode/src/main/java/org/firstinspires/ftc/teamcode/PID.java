package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


public class PID {
    public double kP, kI, kD;
    public double error, sum, lastTime = 0;
    public ElapsedTime runtime;
    public double min_I = -.5;
    public double max_I = .5;

    public PID(double P, double I, double D){
        kP = P;
        kI = I;
        kD = D;
        runtime = new ElapsedTime();
    }

    public double runPID(double goal, double state) {
        double oldError = error;
        error = goal - state;

        double deltaTime = runtime.milliseconds() - lastTime;
        lastTime = runtime.milliseconds();

        //PID Math
        //P math
        double p = kP * error;

        //I math
        if (Math.abs(error) < 15) {
            sum += error * deltaTime;
        } else {
            sum = 0;
        }
        double i = kI * sum;
        i = Range.clip(i, min_I, max_I);

        //D math
        double d = kD * ((error - oldError) / deltaTime);

        //PID calculations
        return (p + i + d);
    }

    public void setConstants(double P, double I, double D) {
        kP = P;
        kI = I;
        kD = D;
    }
}