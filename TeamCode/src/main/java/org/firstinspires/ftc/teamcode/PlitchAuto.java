package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystems.BasicMecanum;
import org.firstinspires.ftc.teamcode.Subsystems.Clamps;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.IMU;
import org.firstinspires.ftc.teamcode.Subsystems.Vision;
import org.firstinspires.ftc.teamcode.Utility.PID;


@Autonomous(name="Skystone Auto", group="Linear Opmode")
public class PlitchAuto extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime runtime2 = new ElapsedTime();
    private Servo servo;
    private IMU imu;
    private BasicMecanum driveTrain;
    int leg = -1;
    private int directionSign = -1;
    @Override
    public void runOpMode() {
        imu = new IMU(hardwareMap);
        driveTrain = new BasicMecanum(imu, hardwareMap);
        servo = hardwareMap.get(Servo.class, "s");

        getDirection();
        // Wait for the match to start
        waitForStart();
        runtime.reset();
        runtime2.reset();
        while (opModeIsActive()) {
            if (leg == -1) {
                driveTrain.closedLoopMecanum(.3 * directionSign,0,0);
                if (runtime.milliseconds() > 1500) {
                    leg++;
                }
            } else if (leg == 0) {
                servo.setPosition(.6);
                runtime.reset();
                leg++;
            } else if (leg == 1) {
                driveTrain.closedLoopMecanum(0,-.3,0);
                if(runtime.milliseconds() > 3000) {
                    leg++;
                    servo.setPosition(1);
                    runtime.reset();
                    driveTrain.closedLoopMecanum(0,0,0);
                }
            } else if (leg == 2) {
                driveTrain.closedLoopMecanum(0,.3,0);
                if(runtime.milliseconds() > 3000) {
                    driveTrain.closedLoopMecanum(0,0,0);
                    leg++;
                }
            } else {
                driveTrain.closedLoopMecanum(0,0,0);
            }
            telemetry.addData("leg", leg);
            telemetry.addData("gds", runtime.milliseconds());
        }
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

}