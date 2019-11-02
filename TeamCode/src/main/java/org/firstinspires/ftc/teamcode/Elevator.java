package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Elevator {
    private DcMotor elevatorMotor;
    private PID elevatorPID;
    private ElapsedTime runtime;

    // State 0 is reserved for IDLE
    // Intake states: 1 - 10
    // Elevator states: 11 - 20
    // Robot States: 21 - 30
    private int state = 0;
    private int idle = 0;
    private int zeroing = 11;
    private int moving = 12;

    int goal = 0;

    boolean zeroed = false;
    private double velocitySetpoint = 0;
    private int lastPos = 0;
    private double lastTime = 0;
    private Double zeroSpeed = .1;

    public Elevator(HardwareMap hardwareMap) {
        elevatorMotor = hardwareMap.get(DcMotor.class, "E");
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevatorPID = new PID(0, 0, 0);
        runtime = new ElapsedTime();
    }

    public void runElevator(int GOAL, double manualPower) {
        if (Math.abs(manualPower) > .05) {
            setPower(manualPower);
            return;
        }

        if (!zeroed) {
            state = zeroing;
        } else if (Math.abs(elevatorMotor.getCurrentPosition() - goal) > 50) {
            state = moving;
            goal = GOAL;
        }

        if (state == idle) {
            setPower(0);
        } else if (state == zeroing) {
            setPower(zeroSpeed);
            if (getVelocity() < velocitySetpoint) {
                elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                setPower(0);
                state = idle;
                zeroed = true;
            }
        } else if (state == moving) {
            setPower(elevatorPID.runPID(goal, elevatorMotor.getCurrentPosition()));
        }
    }

    public void setPower(double power) {
        elevatorMotor.setPower(power);
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

    public double getVelocity() {
        int pos = elevatorMotor.getCurrentPosition();
        double time = runtime.milliseconds();
        double velocity = Math.abs((pos - lastPos) / (time - lastTime));
        lastTime = time;
        lastPos = pos;
        return velocity;
    }
}
