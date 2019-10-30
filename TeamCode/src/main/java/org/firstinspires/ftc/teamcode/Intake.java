package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotor leftIntake, rightIntake;
    private PID intakeLeftArmPid;
    private PID intakeRightArmPid;

    // State 0 is reserved for IDLE
    // Intake states: 1 - 10
    // Elevator states: 11 - 20
    // Robot States: 21 - 30
    private int state = 0;
    private int idle = 0;
    private int running = 1;
    private int opening = 2;
    private int open = 3;

    private int openPos = 500;

    public Intake(HardwareMap hardwareMap) {
        leftIntake = hardwareMap.get(DcMotor.class, "I_L");
        rightIntake = hardwareMap.get(DcMotor.class, "I_R");

        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        leftIntake.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        rightIntake.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));

        intakeLeftArmPid = new PID(0,0,0);
        intakeRightArmPid = new PID(0,0,0);
    }

    public void runIntake(boolean openUp, boolean intakeIn) {
        if (intakeIn) {
            state = running;
        } else if (openUp) {
            state = opening;
        } else if (state != opening) {
            state = idle;
        }

        if (state == idle) {
            setPower(0,0);
        } else if (state == running) {
            setPower(1,1);
        } else if (state == opening) {
            double lPower = intakeLeftArmPid.runPID(openPos, leftIntake.getCurrentPosition());
            double rPower = intakeRightArmPid.runPID(openPos, rightIntake.getCurrentPosition());
            setPower(lPower,rPower);
        }
    }

    public void setState(int STATE) {
        state = STATE;
    }

    public int getState() {
        return state;
    }

    public void setPower(double leftPower, double rightPower) {
        leftIntake.setPower(leftPower);
        rightIntake.setPower(rightPower);
    }
}
