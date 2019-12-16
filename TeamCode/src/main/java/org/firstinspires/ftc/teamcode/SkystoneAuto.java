package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystems.BasicMecanum;
import org.firstinspires.ftc.teamcode.Subsystems.Clamps;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.IMU;
import org.firstinspires.ftc.teamcode.Subsystems.Vision;
import org.firstinspires.ftc.teamcode.Utility.PID;

@Autonomous(name="Skystone Auto", group="Linear Opmode")
public class SkystoneAuto extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime runtime2 = new ElapsedTime();

    private IMU imu;
    private BasicMecanum driveTrain;
    private Elevator elevator;
    private Clamps clamps;
    private Vision vision;

    private PID movePID;
    private PID visXPID;
    private PID visYPID;

    // Vision stuff
    private double xPos = 0;
    private double yPos = 0;
    private double zPos = 0;
    private boolean visActive = false;
    private final double zOffset = 170; // Native units
    private final double xOffset = 0;   // Native units

    // Drive train
    private double xPower = 0;
    private double yPower = 0;
    private int moveGoal = 0;
    private boolean closedLoop = true;
    private double turnPower = 0;
    private double headingGoal = 0;

    // General
    private int leg = -1;
    private int directionSign = -1;

    // Constants
    private final double strafePower = .2;              // Power that the robot will strafe at
    private final double strafeYCompensation = .075;   // Forward/backward compensation power during strafing

    private final int moveError = 20;                   // Number of clicks any Forward/backward movement can be off
    private final int firstDriveDistance = 1250;       // Number of clicks the robot drives forward for at the beginning of auto
    private final int reverseDriveDistance = 500;       // Number of clicks the robot backs up for after it grabs the block
    private final int deliverDriveDistance = 2000;      // Number of clicks the robot drives forward for to deliver the block under the bridge
    private final double maxDrivePower = .3;            // The mac power to send to the drive motors

    private final int turnAngle = 90;                   // The angle to turn to when delivering
    private final double maxTurnPower = .1;             // The maximum power that the robot will turn at
    private final int angleTolerance = 10;              // The angle tolerance when turning

    private final int strafeTime = 1000;                // The amount of time the robot strafe for when looking for a block, before waiting
    private final int waitTime = 500;                   // The amount of time the robot waits for when looking for a block

    private final double maxTrackingPower = .2;         // The maximum power the robot will drive at when centering on a block
    private final int zTolerance = 60;                  // The tolerance the robot can be from the block in the z (native to vision)/ y  (native to robot) direction
    private final int xTolerance = 5;                   // The tolerance the robot can be from the block in the x direction
    private final int inToleranceTime = 1500;           // The time the robot needs to be in the tolerance range before moving on


    @Override
    public void runOpMode() {
        imu = new IMU(hardwareMap);
        elevator = new Elevator(hardwareMap);
        driveTrain = new BasicMecanum(imu, hardwareMap);
        clamps = new Clamps(hardwareMap);
        vision = new Vision(hardwareMap);

        // Vision centering PID
        visXPID = new PID(-.01,0,0);
        visYPID = new PID(.0035,0,0);

        movePID = new PID(.1,0,.05);

        clamps.setState(clamps.runToPos);

        // Get the direction from the user
        getDirection();

        // Wait for the match to start
        waitForStart();
        runtime.reset();
        runtime2.reset();
        while (opModeIsActive()) {

            // Run vision
            if (visActive) {
                vision.runVision();
            }

            // Get the vision data
            xPos = vision.getxPos();
            yPos = vision.getyPos();
            zPos = vision.getzPos();

            // Every leg of the auto
            if (leg == -1) {
                xPower = 1.5 * (strafePower * -directionSign);
                yPower = 1.5 * (strafeYCompensation);
                if(runtime.milliseconds() > 1000) {
                    nextLeg();
                    moveGoal = (int) driveTrain.getFrontRightPos() - firstDriveDistance;
                }
            }
            // Drive forward
            else if (leg == 0) {
                clamps.openArm();

                yPower = Range.clip(movePID.runPID(moveGoal, driveTrain.getFrontRightPos()),-maxDrivePower,maxDrivePower);
                telemetry.addData("thing: ", Math.abs(moveGoal - driveTrain.getFrontRightPos()));
                if (Math.abs(moveGoal - driveTrain.getFrontRightPos()) < moveError) {
                    nextLeg();
                    visActive = true;   // Enable vision tracking
                }
            }
            // Strafe until the target is found, then track it down
            else if (leg == 1) {
                // Check if the target is in view
                if (vision.getInView()) {
                    // Drive to the target
                    xPower = Range.clip(visXPID.runPID(xOffset, xPos),-maxTrackingPower,maxTrackingPower);
                    yPower = Range.clip(visYPID.runPID(zOffset, zPos), -maxTrackingPower, maxTrackingPower);
                    // Check if the target is within tolerance
                    if (Math.abs(zPos) - zOffset < zTolerance && Math.abs(xPos) < xTolerance) {
                        // Check if we have ben in the tolerance for the correct amount of time
                        if (runtime.milliseconds() > inToleranceTime) {
                            // Exit leg, and hook the block
                            // Grab the block
                            clamps.openArm();
                            clamps.hook();
                            // Disable tracking
                            visActive = false;
                            // Set the move goal, so that in the next leg we drive the correct distance
                            moveGoal = (int) driveTrain.getFrontRightPos() + reverseDriveDistance;
                            nextLeg();
                        }
                    } else {
                        runtime.reset();
                    }
                } else {
                    // Strafe right at a interval
                    if (runtime2.milliseconds() < strafeTime) {
                        xPower = strafePower * directionSign;
                        yPower = strafeYCompensation * directionSign;
                    } else if (runtime2.milliseconds() < (strafeTime + waitTime)) {
                        resetPower();
                    } else {
                        runtime2.reset();
                    }
                }
            }
            // Drive backwards
            else if (leg == 2) {
                yPower = Range.clip(movePID.runPID(moveGoal, driveTrain.getFrontRightPos()),-maxDrivePower, maxDrivePower);
                if (Math.abs(moveGoal - driveTrain.getFrontRightPos()) < moveError) {
                    // Set the heading goal to turn next leg
                    headingGoal = turnAngle * directionSign;
                    closedLoop = false;
                    nextLeg();
                }
            }
            // Turn 90 degs
            else if (leg == 3) {
                closedLoop = false;
                turnPower = Range.clip(imu.getYaw() - headingGoal, -maxTurnPower, maxTurnPower);
                if (Math.abs(imu.getYaw()) - turnAngle < angleTolerance) {
                    nextLeg();
                    closedLoop = true;
                    moveGoal = (int) driveTrain.getFrontRightPos() - deliverDriveDistance;
                }
            }
            // Drive under the bridge
            else if (leg == 4) {
                yPower = Range.clip(movePID.runPID(moveGoal, driveTrain.getFrontRightPos()),-maxDrivePower, maxDrivePower);
                if (Math.abs(moveGoal - driveTrain.getFrontRightPos()) < moveError) {
                    nextLeg();
                    runtime.reset();
                }

            }
            // Do nothing
            else if (leg == 5) {
                //resetPower();
                elevator.setGoal(-800);
                elevator.setState(elevator.moving);
                clamps.preClamp();
                yPower = .3;
                if (runtime.milliseconds() > 2500) {
                    nextLeg();
                }
            } else if (leg == 6) {
                resetPower();
            }

            // Run the subsystems
            if (closedLoop) {
                driveTrain.closedLoopMecanum(xPower, yPower, headingGoal);
            } else {
                telemetry.addData("Turning: ", turnPower);
                driveTrain.turn(.05 * directionSign);
            }
            clamps.runClamps();
            elevator.runElevator(0);

            telemetry.addData("Turn Power: ", turnPower);
            telemetry.addData("Leg: ", leg);
            telemetry.addData("x-pos", xPos);
            telemetry.addData("y-pos", yPos);
            telemetry.addData("z-pos", zPos);
            telemetry.update();
        }
        // Turn of the light on the phone
        vision.disableLight();
    }
    private void getDirection() {
        // Get the direction of the auto:
        while (!opModeIsActive()) {
            // Let the user decide which direction to drive in auto
            if (gamepad1.dpad_left) {
                directionSign = 1;
            } else if (gamepad1.dpad_right) {
                directionSign = -1;
            } else if (gamepad1.x) {
                break;
            }

            // Update telemetry with the users input
            if (directionSign == -1) {
                telemetry.addData("Move:", "Right");
            } else {
                telemetry.addData("Move:", "Left");
            }
            telemetry.update();
        }
        // Update telemetry with the users input
        if (directionSign == -1) {
            telemetry.addData("Move:", "Right");
        } else {
            telemetry.addData("Move:", "Left");
        }
        telemetry.addData("Set", "");
        telemetry.update();
    }
    private void resetPower() {
        xPower = 0;
        yPower = 0;
        turnPower = 0;
    }
    private void nextLeg() {
        leg++;
        resetPower();
    }
}