package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystems.BasicMecanum;
import org.firstinspires.ftc.teamcode.Subsystems.Clamps;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.FoundationPuller;
import org.firstinspires.ftc.teamcode.Subsystems.IMU;
import org.firstinspires.ftc.teamcode.Subsystems.Vision;
import org.firstinspires.ftc.teamcode.Utility.PID;


@Autonomous(name="PULL", group="Linear Opmode")
public class PullyThingAuto extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime runtime2 = new ElapsedTime();

    private FoundationPuller foundationPuller;
    private IMU imu;
    private BasicMecanum driveTrain;
    private Elevator elevator;
    private Clamps clamps;
    private Vision vision;

    private PID movePID;
    private PID visXPID;
    private PID visYPID;


    // General
    private int leg = 096;
    private int directionSign = -1;


    @Override
    public void runOpMode() {
        imu = new IMU(hardwareMap);
        elevator = new Elevator(hardwareMap);
        driveTrain = new BasicMecanum(imu, hardwareMap);
        foundationPuller = new FoundationPuller(hardwareMap);

        // Get the direction from the user
        getDirection();

        // Wait for the match to start
        waitForStart();
        runtime.reset();
        runtime2.reset();
        while (opModeIsActive()) {

            if (leg == -1) {
                driveTrain.closedLoopMecanum(.3 * directionSign,0,0);
                if (runtime.milliseconds() > 1500) {
                    runtime.reset();
                    nextLeg();
                }
            }
            // Every leg of the auto
            else if (leg == 0) {
                driveTrain.closedLoopMecanum(0,.5,0);
                if(runtime.milliseconds() > 1000) {
                    nextLeg();
                    runtime.reset();
                }
            }
            // Drive forward
            else if (leg == 1) {
                driveTrain.closedLoopMecanum(0,0,0);
                foundationPuller.openPuller();
                if(runtime.milliseconds() > 3000) {
//                    nextLeg();
                    runtime.reset();
                }
            }
            // Strafe until the target is found, then track it down
            else if (leg == 2) {
                driveTrain.closedLoopMecanum(0,-.3,5 * directionSign);
                if (runtime.milliseconds() > 4000) {
                    nextLeg();
                    runtime.reset();
                    foundationPuller.prePuller();
                    elevator.setGoal(-800);
                }
            }
            else if (leg == 3) {
                driveTrain.closedLoopMecanum(-.3 * directionSign,0,0);
                if (runtime.milliseconds() > 3000) {
                    runtime.reset();
                    nextLeg();
                }
            }
            else if (leg == 4) {
                driveTrain.closedLoopMecanum(0,0,0);
            }
            elevator.runElevator(0);

//            telemetry.addData("Turn Power: ", turnPower);
//            telemetry.update();
        }
        // Turn of the light on the phone
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
    private void nextLeg() {
        leg++;
    }
}