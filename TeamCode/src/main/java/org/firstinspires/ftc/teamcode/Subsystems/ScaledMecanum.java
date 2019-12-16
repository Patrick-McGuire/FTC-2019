package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class ScaledMecanum {
    // Class members
    private IMU imu;
    private Gamepad gamepad1;
    private Gamepad gamepad2;
    private ElapsedTime runtime;
    BasicMecanum driveTrain;      // Public so that instances can chose to directly control the drivetrain (and so I don't have to write a bunch of code)

    // Vars to keep track of everything need to control the drivetrain
    private double headingGoal;

    private double xPower = 0;
    private double yPower = 0;
    private double tPower = 0;

    private double xSign = 0;
    private double ySign = 0;
    private double tSign = 0;

    private double gamepadX;
    private double gamepadY;
    private double gamepadT;

    // Scaling info
    private boolean clipPower = true;

    private double maxPower = .4;

    private final double slowRangeScale = .8;
    private final double noRangeScale = .9;
    private double rangeScale = slowRangeScale;   // The range that the mins and maxes below will be scaled from

    private final double xSlowRangeMin = .1;
    private final double xSlowRangeMax = .3;

    private final double ySlowRangeMin = .05;
    private final double ySlowRangeMax = .25;

    private final double tSlowRangeMin = .05;
    private final double tSlowRangeMax = .25;

    // Constructor
    public ScaledMecanum(IMU imu, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        this.imu = imu;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        driveTrain = new BasicMecanum(imu, hardwareMap);
        runtime = new ElapsedTime();
    }

    // Main method: call in a loop to run the drive train based on inputs from gamepad1
    public void runMecanum() {
        getScaledValues();

        // Drive train code
        if(runtime.milliseconds() < 300) {                                    // Reset heading goal to current heading for 300 milliseconds after the joystick is released
            headingGoal = imu.getYaw();
        }
        if(Math.abs(imu.getYaw() - headingGoal) > 45) {                       // If we are way of our desired heading, reset heading goal to current heading
            headingGoal = imu.getYaw();
        }
        if(gamepad1.right_stick_x > .05 || gamepad1.right_stick_x < -.05) {   // If we are telling it to turn, turn at the specified power
            driveTrain.openLoopMecanum(xPower, yPower, tPower);
            runtime.reset();
        } else {                                                              // Else hold heading
            driveTrain.closedLoopMecanum(xPower, yPower, headingGoal);
        }
    }

    // Scales turn, x, and y power in one of two ways:
    // If the joystick input is less than a setpoint (rangeScale):
    //      scale the joystick value from 0 - setpoint to be between two set values (SlowRangeMin, SlowRangeMax)
    // If the joystick input is greater than the setpoint:
    //      scale the joystick value from setpoint - 1 to be between SlowRangeMax and 1
    // This gets slow increase of power from 0 - setpoint on the joystick, and a fast increase between setpoint - 1
    // (with the current setpoint and SlowRangeMins SlowRangeMaxes)
    private void getScaledValues() {
        // Get the joystick values for the drivetrain
        gamepadX = -gamepad1.left_stick_x;
        gamepadY = gamepad1.left_stick_y;
        gamepadT = gamepad1.right_stick_x;

        // Get the direction of the joysticks
        xSign = Math.abs(gamepadX) / gamepadX;
        ySign = Math.abs(gamepadY) / gamepadY;
        tSign = Math.abs(gamepadT) / gamepadT;

        // Scale the x power
        if (Math.abs(gamepadX) > 0) {
            gamepadX = Math.abs(gamepadX);
            if (gamepadX < rangeScale) {
                // Scale the gamepad from 0 - rangeScale to 0 - 1
                gamepadX = gamepadX * (1 / rangeScale);
                // Scale the power from slowRangeMin to slowRangeMax
                xPower = (gamepadX * (xSlowRangeMax - xSlowRangeMin)) + xSlowRangeMin;
            } else {
                // Scale the gamepad from rangeScale - 1 to 0 - 1
                gamepadX = (gamepadX - rangeScale) * (1 / (1 - rangeScale));
                // Scale the power from slowRangeMax to 1
                xPower = (gamepadX * (1 - xSlowRangeMax) + xSlowRangeMax);
            }
            xPower = xPower * xSign;
        } else {
            xPower = 0;
        }

        // Scale the y power
        if (Math.abs(gamepadY) > 0) {
            gamepadY = Math.abs(gamepadY);
            if (gamepadY < rangeScale) {
                // Scale the gamepad from 0 - rangeScale to 0 - 1
                gamepadY = gamepadY * (1 / rangeScale);
                // Scale the power from slowRangeMin to slowRangeMax
                yPower = (gamepadY * (ySlowRangeMax - ySlowRangeMin)) + ySlowRangeMin;
            } else {
                // Scale the gamepad from rangeScale - 1 to 0 - 1
                gamepadY = (gamepadY - rangeScale) * (1 / (1 - rangeScale));
                // Scale the power from slowRangeMax to 1
                yPower = (gamepadY * (1 - ySlowRangeMax) + ySlowRangeMax);
            }
            yPower = yPower * ySign;
        } else {
            yPower = 0;
        }

        // Scale the t (turn) power
        if (Math.abs(gamepadT) > 0) {
            gamepadT = Math.abs(gamepadT);
            if (gamepadT < rangeScale) {
                // Scale the gamepad from 0 - rangeScale to 0 - 1
                gamepadT = gamepadT * (1 / rangeScale);
                // Scale the power from slowRangeMin to slowRangeMax
                tPower = (gamepadT * (tSlowRangeMax - tSlowRangeMin)) + tSlowRangeMin;
            } else {
                // Scale the gamepad from rangeScale - 1 to 0 - 1
                gamepadT = (gamepadT - rangeScale) * (1 / (1 - rangeScale));
                // Scale the power from slowRangeMax to 1
                tPower = (gamepadT * (1 - tSlowRangeMax) + tSlowRangeMax);
            }
            tPower = tPower * tSign;
        } else {
            tPower = 0;
        }
        if (clipPower) {
            xPower = Range.clip(xPower, -maxPower, maxPower);
            yPower = Range.clip(yPower, -maxPower, maxPower);
            tPower = Range.clip(tPower, -maxPower, maxPower);
        }

    }

    public void enableYeet() {
        rangeScale = slowRangeScale;
        clipPower = false;
    }

    public void disableYeet() {
        rangeScale = noRangeScale;
        clipPower = true;
    }
    public BasicMecanum getDrivetrain() {
        return driveTrain;
    }
}