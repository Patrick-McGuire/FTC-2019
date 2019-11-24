package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Elevator {
    // Declare class members.
    private DcMotor elevatorMotor;
    private PID elevatorPID;
    private ElapsedTime runtime;
    private ElapsedTime runTime2;
    private ElapsedTime runTime3;

    // State 0 is reserved for IDLE
    // Intake states: 1 - 10
    // Elevator states: 11 - 20
    // Robot States: 21 - 30
    private int prevState = 0;
    private int state = 0;
    private final int idle = 0;
    private final int zeroing = 11;
    private final int moving = 12;
    private final int quickZero = 13;
    private final int delayedMove = 14;

    // Elevator specs
    private final int maxHeight = -3000;
    private final int minHeight = 50;
    private final int bumpHeight = 300;
    private final int blockHeight = -650;
    private final int setpointTolerance = 10;

    // Zeroing stuff
    private final double velocitySetpoint = 0;
    private final double zeroSpeed = .25;
    private double velo = 0;
    private boolean zeroed = false;
    private int lastPos = 0;
    private double lastTime = 0;

    // Ele goal stuff
    private int goal = 0 - minHeight;
    private int refGoal = -490 - minHeight;
    private int refGoal2 = 1000;
    private int nextGoal = 0;
    private double stateDelayTime = 3000;

    // For dampening at ends
    private double errorFromEnds = 0;

    public Elevator(HardwareMap hardwareMap) {
        // Init the motor, and set it to track it's encoder
        elevatorMotor = hardwareMap.get(DcMotor.class, "ELE");
        elevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // PID controller that will run the elevator
        elevatorPID = new PID(.003,0, .008);

        // Runtime keeps track of various times (for velo, delays, etc...)
        runtime = new ElapsedTime();
        runTime2 = new ElapsedTime();
        runTime3 = new ElapsedTime();
    }

    // Main method
    // Call in a loop. Runs the elevator
    public void runElevator(double manualPower) {
        // Calculate the velocity of the elevator (from the last time this was called)
        velo = calculateVelocity();

        // If we are overriding the state machene with a manual input, run at specified power
        if (Math.abs(manualPower) > .05) {
            setPower(manualPower, false);
            goal = elevatorMotor.getCurrentPosition();
            return;
        }

        // Limit the goal to between our max and min height
        goal = Range.clip(goal, maxHeight, -minHeight);

        // Make sure the elevator is zeroed
        if (!zeroed) {
            state = zeroing;
        }

        // Keep track of how long we spend in the zeroing state
        if (prevState != state && state == zeroing) {
            runTime2.reset();
        }

        // Keep track of how long we spend in the delayed move state
        if (prevState != state && state == delayedMove) {
            runTime3.reset();
        }

        // Keep track if the state has changed
        prevState = state;

        if (state == moving && Math.abs(getPos() - goal) < setpointTolerance) {             // If the elevator is within tolerance, set state to idle
            state = idle;
        } else if (state == idle && Math.abs(getPos() - goal) >= setpointTolerance) {       // If the elevator is not within tolerance, set state to moving
            state = moving;
        }

        // State machine
        if (state == idle) {                                                                        // If in idle, set the motor's power to 0
            setPower(0,false);
        } else if (state == zeroing) {
            setPower(zeroSpeed,false);                                                         // If in zeroing, drive the elevator down until velocity is 0. Then reset the ele encoder and set the state to moving
            if (velo <= velocitySetpoint && runTime2.milliseconds() > 100) {
                elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                setPower(0,false);
                state = moving;
                zeroed = true;
            }
        } else if (state == moving) {                                                               // If in moving, use PID to move elevator to goal
            setPower(elevatorPID.runPID(goal, elevatorMotor.getCurrentPosition()), false);
        } else if (state == quickZero) {                                                            // If in quick Zero, run the elevator to 0, then set state to zeroing
            goal = -minHeight;
            setPower(elevatorPID.runPID(goal, elevatorMotor.getCurrentPosition()), false);
            if (getPos() > -100) {
                state = zeroing;
            }
        } else if (state == delayedMove) {                                                          // If delayed move Wait for a amount of time, then have the elevator go to the nex position
            setPower(elevatorPID.runPID(goal, elevatorMotor.getCurrentPosition()), false);
            if (runTime3.milliseconds() > stateDelayTime) {
                state = moving;
                goal = nextGoal;
                release();
            }
        }
    }

    // Private methods
    // Calculate the velocity of the elevator from the last time this was called
    private double calculateVelocity() {
        int pos = elevatorMotor.getCurrentPosition();
        double time = runtime.milliseconds();
        double velocity = Math.abs((pos - lastPos) / (time - lastTime));
        lastTime = time;
        lastPos = pos;
        return velocity;
    }

    // Set the ele motors power
    private void setPower(double power, boolean limit) {
        if (limit) {
            errorFromEnds = Math.min(Math.abs(getPos()), Math.abs(getPos() - maxHeight));
            errorFromEnds = Range.clip(errorFromEnds * .001, -1, 1);
            elevatorMotor.setPower(Math.max(Math.min(Math.abs(power), Math.abs(errorFromEnds)), .1) * Math.abs(power) / power);
        } else {
            elevatorMotor.setPower(Range.clip(power, -1, 1));
        }
    }

    // Public methods
    // Move the goal up one block height from the last block
    public void goalUpBlock() {
        this.goal = refGoal + blockHeight;
        refGoal = this.goal;
    }

    // Move the goal down one block height from the last block
    public void goalDownBlock() {
        this.goal = refGoal - blockHeight;
        refGoal = this.goal;
    }

    // Go to last block height
    public void runGoal() {
        this.goal = refGoal;
    }

    // Move to a goal, after a delay
    public void delayedMove(int nextGoal, double waitTime) {
        stateDelayTime = waitTime;
        this.nextGoal = nextGoal;
        state = delayedMove;
        runTime3.reset();
    }

    // Move the elevator down bump height
    public void place() {
        if (refGoal2 == 1000) {
            refGoal2 = goal;
            goal = goal + bumpHeight;
        }
    }

    // Move the elevator up bump height
    public void release() {
        if (refGoal2 != 1000) {
            goal = refGoal2;
            refGoal2 = 1000;
        }
    }

    // Getter methods
    public int getState() {
        return state;
    }
    public int getPos() {
        return elevatorMotor.getCurrentPosition();
    }
    public int getGoal() {
        return goal;
    }
    public double getVelocity() {
        return velo; // Ticks/millisecond
    }
    public double getPower() {
        return elevatorMotor.getPower();
    }

    // Setter methods
    public void setState(int STATE) {
        state = STATE;
    }
    public void setGoal(int goal) {
        this.goal = goal;
    }
}