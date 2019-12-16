package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Clamps;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.Extension;
import org.firstinspires.ftc.teamcode.Subsystems.IMU;
import org.firstinspires.ftc.teamcode.Subsystems.ScaledMecanum;

// Two controller teleop mode
@TeleOp(name = "Teleop 2C", group = "Opmode")
public final class Teleop extends OpMode {
    // Declare OpMode members.
    private IMU imu;
    private Elevator elevator;
    private Clamps clamps;
    private Extension extension;
    private ScaledMecanum drivetrain;
    private DigitalChannel digitalTouch;
    private Servo armServo;

    // Init vars
    private boolean pressLastTime = false;

    @Override
    public final void init() {
        // Init subsystems
        imu = new IMU(hardwareMap);
        elevator = new Elevator(hardwareMap);
        clamps = new Clamps(hardwareMap);
        extension = new Extension((hardwareMap));
        drivetrain = new ScaledMecanum(imu, hardwareMap, gamepad1, gamepad2);
        clamps.closeArm();
        clamps.setState(clamps.runToPos);
        digitalTouch = hardwareMap.get(DigitalChannel.class, "ls");
        armServo = hardwareMap.get(Servo.class, "s5");
        armServo.setPosition(.5);
    }

    @Override
    public final void loop() {
        if (gamepad1.b) {
            armServo.setPosition(0);
        } else if (gamepad1.y) {
            armServo.setPosition(.5);
            clamps.capPlace();
        }
        if (gamepad1.dpad_right) {
            clamps.openArm();
        }
        if (gamepad1.dpad_left) {
            clamps.closeArm();
        }

        if (digitalTouch.getState()) {
            telemetry.addData("Digital Touch", "Is Not Pressed");
        } else {
            telemetry.addData("Digital Touch", "Is Pressed");
        }

        if (elevator.getPos() < -200 && !(elevator.getState() == elevator.quickZero)) {
            drivetrain.disableYeet();
        } else {
            drivetrain.enableYeet();
        }

        drivetrain.runMecanum();

        // Elevator/clamps code
        // Get the inputs from the controller
        boolean gp2x = gamepad2.x;
        boolean gp2a = gamepad2.a;
        boolean gp2b = gamepad2.b;
        boolean gp2y = gamepad2.y;

        if (gp2x && !pressLastTime){                    // brings the elevator to zero, then zeros it
            elevator.setState(elevator.quickZero);
            pressLastTime = true;
        } else if (gp2a && !pressLastTime){             // moves the elevator to one block lower than the last block height
            elevator.goalDownBlock();
            pressLastTime = true;
        } else if (gp2b && !pressLastTime) {            // moves the elevator to the last block height
            elevator.runGoal();
            pressLastTime = true;
        } else if (gp2y && !pressLastTime) {            // moves the elevator to one block higher than the last block height
            elevator.goalUpBlock();
            pressLastTime = true;
        } else if (gp2x || gp2a || gp2b || gp2y) {      // Keep track of whether the button was just pressed
            pressLastTime = true;
        } else {                                        // Keep track of whether the button was just pressed
            pressLastTime = false;
        }
        if (gamepad2.dpad_down || gamepad1.dpad_down) {
            elevator.setState(elevator.zeroing);
        }

        if (gamepad2.right_bumper) {
            elevator.setGoal(-1918);
        }

        // Placing block
        if (gamepad1.right_trigger > .05) {
            if (gamepad1.right_trigger < .8) {              // Move the elevator down the height of the bumps on top of the stones
                elevator.place();
            } else if (gamepad1.right_trigger >= .8){       // Release the clamps, and int .5 seconds move the elevator back up
                elevator.delayedMove(elevator.getGoal(), 500);
                clamps.preClamp();
                clamps.setState(clamps.runToPos);
            }
        } else if (elevator.getState() != elevator.delayedMove){    // Move the elevator back up
            elevator.release();
        }

        // Clamps
        if (gamepad1.right_bumper) {
            clamps.clamp();
            clamps.setState(clamps.runToPos);
        } else if(gamepad1.left_bumper) {
            clamps.preClamp();
            clamps.setState(clamps.runToPos);
        } else if (gamepad1.x) {
            clamps.open();
            clamps.setState(clamps.runToPos);
        } else if (gamepad1.left_trigger > 0) {
            clamps.close();
            clamps.setState(clamps.runToPos);
        }

        // Run the horizontal extension
        extension.runExtension(gamepad2.left_stick_y);
        // Run the clamps
        clamps.runClamps();
        // Run the elevator
        elevator.runElevator(gamepad2.right_stick_y);

        // Telemetry to the driver station. NOTE: %.2f means floating point
        telemetry.addData("Robot","Heading (%.2f)",
                imu.getYaw());
        telemetry.addData("Elevator", "Velo (%.2f), Pos (%.2f), state (%.2f), goal (%.2f), power (%.2f)",
                elevator.getVelocity(), (double) elevator.getPos(), (double) elevator.getState(), (double) elevator.getGoal(), elevator.getPower());
        telemetry.addData("Clamp","State (%.2f), Pos (%.2f)",
                (double) clamps.getState(), clamps.getPositionGoal());
        telemetry.addData("Drive Train","F-Left (%.2f), F-Right (%.2f), B-Left (%.2f), Back Right (%.2f)",
                drivetrain.getDrivetrain().getFrontLeftPower(), drivetrain.getDrivetrain().getFrontRightPower(), drivetrain.getDrivetrain().getBackLeftPower(), drivetrain.getDrivetrain().getBackRightPower());
    }
}