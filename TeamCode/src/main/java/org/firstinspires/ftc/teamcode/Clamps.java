package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Clamps {
    private CRServo leftClamp;
    private CRServo rightClamp ;

    private ElapsedTime runtime;

    private double clampPower = -1;
    private double openPower = .5;
    private double zeroPower = -.5;
    private int zeroTime = 2000; // Milliseconds
    private int openTime = 3000; // Milliseconds
    private int releaseTime = 300; // Milliseconds

    private double refTime = 0;

    // State 0 is reserved for IDLE
    // Intake states: 1 - 10
    // Elevator states: 11 - 20
    // Robot States: 21 - 30
    // Clamp states: 31-40
    private int clampState = 0;
    private int prevClampState = -1;
    private int idle = 0;
    private int openingClamp = 31;
    private int clamping = 32;
    private int zeroingClamp = 33;
    private int releasingClamp = 34;

    public Clamps(HardwareMap hardwareMap) {
        leftClamp = hardwareMap.get(CRServo.class, "s2");
        rightClamp = hardwareMap.get(CRServo.class, "s3");
        runtime = new ElapsedTime();
        clampState = zeroingClamp;
    }

    public void runClamps() {
        if(clampState != prevClampState) {
            refTime = runtime.milliseconds();
        }

        if (clampState == idle) {
            setClampPower(0);
        } else if (clampState == releasingClamp) {
            setClampPower(openPower);
            if (runtime.milliseconds() - refTime > releaseTime) {
                clampState = idle;
            }
        } else if(clampState == clamping) {
            setClampPower(clampPower);
        } else if (clampState == zeroingClamp) {
            setClampPower(zeroPower);
            if (runtime.milliseconds() - refTime > zeroTime) {
                clampState = openingClamp;
            }
        } else if (clampState == openingClamp) {
            setClampPower(openPower);
            if (runtime.milliseconds() - refTime > openTime) {
                clampState = idle;
            }
        } else {
            clampState = idle;
        }
        prevClampState = clampState;
    }


    private void setClampPower(double power) {
        leftClamp.setPower(power);
        rightClamp.setPower(-power);
    }

    private void setClampPower(double lPower, double rPower) {
        leftClamp.setPower(lPower);
        rightClamp.setPower(rPower);
    }

    // Getter and setter
    public void setClampState(int state) {
        this.clampState = state;
    }
    public int getClampState() {
        return clampState;
    }
    public double getClampPower() {
        return Math.max(leftClamp.getPower(), rightClamp.getPower());
    }
    public double getClampStateTime() {
        return (runtime.milliseconds() - refTime); // Milliseconds
    }
}
