package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Clamps {
    private Servo leftClamp;
    private Servo rightClamp;
    private Servo armServo;


    private final double extendArmPos = .26;
    private final double zeroArmPos = 0;
    private boolean openArm = false;
    private boolean firstTime = true;
    private double downTime = 1100;

    private double refTime = 0;

    // Position setpoints
    private final double minPos = .05;
    private final double maxPos = .89;
    private final double preClampPos = .43;
    private final double hookPos = .67;
    private final double capPos = .5;
    private final double clampPos = .72;
    private double positionGoal = preClampPos;

    // State 0 is reserved for IDLE
    // Intake states: 1 - 10
    // Elevator states: 11 - 20
    // Robot States: 21 - 30
    // Clamp states: 31-40
    private int state;
    private int prevState = -1;
    public final int idle = 0;
    public final int runToPos = 1;

    public Clamps(HardwareMap hardwareMap) {
        leftClamp = hardwareMap.get(Servo.class, "s2");
        rightClamp = hardwareMap.get(Servo.class, "s3");
        armServo = hardwareMap.get(Servo.class, "s4");

    }

    public void runClamps() {

        if (openArm) {
            armServo.setPosition(extendArmPos);
        } else {
            armServo.setPosition(zeroArmPos);
        }

        if (state == idle) {
            setPos(minPos);
        } else if (state == runToPos) {
            setPos(positionGoal);
        }
        prevState = state;
    }


    private void setPos(double pos) {
        leftClamp.setPosition(1 - pos);
        rightClamp.setPosition(pos);
    }


    public void openArm() {
        openArm = true;
    }

    public void closeArm() {
        openArm = false;
        armServo.setPosition(zeroArmPos);
    }

    public void open() {
        positionGoal = minPos;
        openArm = false;
    }

    public void close() {
        positionGoal = maxPos;
        openArm = false;
    }

    public void clamp() {
        positionGoal = clampPos;
        openArm = false;
    }

    public void capPlace() {
        positionGoal = capPos;
    }

    public void hook() {
        positionGoal = hookPos;
        openArm = true;
    }

    public void preClamp() {
        positionGoal = preClampPos;
        openArm = false;
    }

    // Getter and setter
    public void setState(int state) {
        this.state = state;
    }

    public int getState() {
        return state;
    }

    public double getPositionGoal() {
        return positionGoal;
    }
}
