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

    private double errorFromEnds = 0;

    int zeroPos = 50;
    private int refGoal = -490 - zeroPos;
     int refGoal2 = 1000;
    private int goal = 0 - zeroPos;
    private int blockHeight = -650;
    private int bumpHeight = 300;
    private int maxHeight = -3100;

    double velo = 0;
    boolean zeroed = false;
    private double velocitySetpoint = 0;
    private int lastPos = 0;
    private double lastTime = 0;
    private double zeroSpeed = .25;

    public Elevator(HardwareMap hardwareMap) {
        elevatorMotor = hardwareMap.get(DcMotor.class, "ELE");
        //
        // elevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elevatorPID = new PID(.003,0, .008);
        runtime = new ElapsedTime();
        runTime2 = new ElapsedTime();
    }

    public void runElevator(double manualPower) {
        velo = calculateVelocity();

        if (Math.abs(manualPower) > .05) {
            setPower(manualPower, false);
            goal = elevatorMotor.getCurrentPosition();
            return;
        }
        goal = Range.clip(goal, maxHeight, -zeroPos);
        if (!zeroed) {
            state = zeroing;
        }
        if (prevState != state && state == zeroing) {
            runTime2.reset();
        }
        if (state == moving && Math.abs(getPos() - goal) < 10) {
            state = idle;
        } else if (state != zeroing && Math.abs(getPos() - goal) >= 10) {
            state = moving;
        }
        if (state == idle) {
            setPower(0,false);
        } else if (state == zeroing) {
            setPower(zeroSpeed,false);
            if (velo <= velocitySetpoint && runTime2.milliseconds() > 100) {
                elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                setPower(0,false);
                state = moving;
                zeroed = true;
            }
        } else if (state == moving) {
            setPower(elevatorPID.runPID(goal, elevatorMotor.getCurrentPosition()), false);
        }
        prevState = state;
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
    public double getPower() {
        return elevatorMotor.getPower();
    }

    private double calculateVelocity() {
        int pos = elevatorMotor.getCurrentPosition();
        double time = runtime.milliseconds();
        double velocity = Math.abs((pos - lastPos) / (time - lastTime));
        lastTime = time;
        lastPos = pos;
        return velocity;
    }

    public void setPower(double power, boolean limit) {
        if (limit) {
            errorFromEnds = Math.min(Math.abs(getPos()), Math.abs(getPos() - maxHeight));
            errorFromEnds = Range.clip(errorFromEnds * .001, -1, 1);
            elevatorMotor.setPower(Math.max(Math.min(Math.abs(power), Math.abs(errorFromEnds)), .1) * Math.abs(power) / power);
        } else {
            elevatorMotor.setPower(Range.clip(power, -1, 1));
        }
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
    public int getGoal() {
        return goal;
    }
    public void place() {
        if (refGoal2 == 1000) {
            refGoal2 = goal;
            goal = goal + bumpHeight;
        }
    }
    public void release() {
        if (refGoal2 != 1000) {
            goal = refGoal2;
            refGoal2 = 1000;
        }
    }
    public void resetPlace() {
        if (refGoal2 != 1000) {
            refGoal2 = 1000;
        }
    }
    public double getVelocity() {
        return velo; // Ticks/millisecond
    }
}
