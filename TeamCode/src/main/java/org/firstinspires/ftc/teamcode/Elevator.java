package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;


public class Elevator {
    DcMotor elevatorMotor;
    private PID elevatorPID;
    private ElapsedTime runtime;
    private ElapsedTime runTime2;

    // State 0 is reserved for IDLE
    // Intake states: 1 - 10
    // Elevator states: 11 - 20
    // Robot States: 21 - 30
    private int prevState = 0;
    private int state = 0;
    private int idle = 0;
    private int zeroing = 11;
    private int moving = 12;

    int zeroPos = 50;
    private int refGoal = 0;
    private int goal = 0 - zeroPos;
    private int blockHeight = -300;

    double velo = 0;
    boolean zeroed = false;
    private double velocitySetpoint = 0;
    private int lastPos = 0;
    private double lastTime = 0;
    private double zeroSpeed = .15;

    public Elevator(HardwareMap hardwareMap) {
        elevatorMotor = hardwareMap.get(DcMotor.class, "ELE");
        elevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elevatorPID = new PID(.005,0, .008);
        runtime = new ElapsedTime();
        runTime2 = new ElapsedTime();
    }

    public void runElevator(double manualPower) {
        if (Math.abs(manualPower) > .05) {
            setPower(manualPower);
            goal = elevatorMotor.getCurrentPosition();
            return;
        }
        goal = Range.clip(goal, -2950, -zeroPos);
        if (!zeroed) {
            state = zeroing;
        }
        if (prevState != state && state == zeroing) {
            runTime2.reset();
        }

        if (state == idle) {
            setPower(0);
        } else if (state == zeroing) {
            velo = getVelocity();
            setPower(zeroSpeed);
            if (velo <= velocitySetpoint && runTime2.milliseconds() > 100) {
                elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                setPower(0);
                state = moving;
                zeroed = true;
            }
        } else if (state == moving) {
            setPower(elevatorPID.runPID(goal, elevatorMotor.getCurrentPosition()));
        }
        prevState = state;
    }

    public void setPower(double power) {
        elevatorMotor.setPower(Range.clip(power,-1,1));
    }

    public void setState(int STATE) {
        state = STATE;
    }

    public int getState() {
        return state;
    }

    public int getPos() {
        return elevatorMotor.getCurrentPosition();
    }

    public void setGoal(int goal) {
        this.goal = goal;
    }
    public void goalUpBlock() {
        this.goal = refGoal + blockHeight;
        refGoal = this.goal;
    }
    public void goalDownBlock() {
        this.goal = refGoal - blockHeight;
        refGoal = this.goal;
    }
    public void runGoal() {
        this.goal = refGoal;
    }

    public double getVelocity() {
        int pos = elevatorMotor.getCurrentPosition();
        double time = runtime.milliseconds();
        double velocity = Math.abs((pos - lastPos) / (time - lastTime));
        lastTime = time;
        lastPos = pos;
        return velocity;
    }
}
