package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

// Two controller teleop mode
@TeleOp(name = "Teleop 2C", group = "Opmode")
public final class Teleop extends OpMode {
    // Declare OpMode members.
    private IMU imu;
    private BasicMecanum driveTrain;
    private Elevator elevator;
    private ElapsedTime runtime = new ElapsedTime();
    private EndEffector endEffector;

    // Init vars
    double headingGoal = 0;
    boolean pressLastTime = false;

    // State 0 is reserved for IDLE
    // Intake states: 1 - 10
    // Elevator states: 11 - 20
    // Robot States: 21 - 30
    // Clamp states: 31-40
    private int state = 0;
    private int idle = 0;
    // Intake States
    private int running = 1;
    private int opening = 2;
    private int open = 3;
    // Elevator states
    private int zeroing = 11;
    private int moving = 12;
    private int quickZero = 13;
    private int delayedState = 14;
    // Robot states
    private int intaking = 21;
    private int handingOff = 22;
    private int holdingStone = 23;
    private int placing = 24;
    // Clamp states
    private int openingClamp = 31;
    private int clamping = 32;
    private int zeroingClamp = 33;
    private int releasingClamp = 34;

    @Override
    public final void init() {
        // Init subsystems
        imu = new IMU(hardwareMap);
        elevator = new Elevator(hardwareMap);
        driveTrain = new BasicMecanum(imu, hardwareMap);
        endEffector = new EndEffector(hardwareMap);
    }

    @Override
    public final void loop() {
        // Get the joystick values for the drivetrain
        double gamepadX = gamepad1.left_stick_x;
        double gamepadY = gamepad1.left_stick_y;
        double gamepadTurn = gamepad1.right_stick_x;

        // Scale power values
        double xPower = -Math.pow(gamepadX, 2) * (Math.abs(gamepadX) / (gamepadX + .0001));
        double yPower = Math.pow(gamepadY, 2) * (Math.abs(gamepadY) / (gamepadY + .0001));
        double turnPower = Math.pow(gamepadTurn, 2) * (Math.abs(gamepadTurn) / (gamepadTurn + .0001));

        // Drive train code
        if(runtime.milliseconds() < 300) {
            headingGoal = imu.getYaw();
        }
        if(Math.abs(imu.getYaw() - headingGoal) > 30) {                       // Keep track of the heading to hold for 300 milliseconds after the joystick is released
            headingGoal = imu.getYaw();
        }
        if(gamepad1.right_stick_x > .05 || gamepad1.right_stick_x < -.05) {   // If we are telling it to turn, turn at the specifyed power
            driveTrain.openLoopMecanum(xPower, yPower, turnPower);
            runtime.reset();
        } else {                                                              // Else hold heading
            driveTrain.closedLoopMecanum(xPower, yPower, headingGoal);
        }

        // Elevator/clamps code
        // Get the inputs from the controller
        boolean gp2x = gamepad2.x;
        boolean gp2a = gamepad2.a;
        boolean gp2b = gamepad2.b;
        boolean gp2y = gamepad2.y;

        if (gp2x && !pressLastTime){                    // brings the elevator to zero, then zeros it
            elevator.setState(quickZero);
            endEffector.setClampState(zeroingClamp);
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
        if (gamepad2.dpad_down) {
            elevator.setState(zeroing);
        }

        // Placing block
        if (gamepad1.right_trigger > .05) {
            if (gamepad1.right_trigger < .8) {              // Move the elevator down the height of the bumps on top of the stones
                elevator.place();
            } else if (gamepad1.right_trigger >= .8){       // Release the clamps, and 3 seconds move the elevator back up
                elevator.delayedMove(elevator.getGoal(), 3000);
                endEffector.setClampState(releasingClamp);
            }
        } else if (elevator.getState() != delayedState){    // Move the elevator back up
            elevator.release();
        }

        // Clamps
        if (gamepad1.right_bumper) {                        // Clamp on the block at 100% power
            endEffector.setClampState(clamping);
        } else if(gamepad1.left_bumper) {
            endEffector.setClampState(releasingClamp);      // Move the clamps out for small amount of time
        } else if (gamepad1.x) {
            endEffector.setClampState(zeroingClamp);        // Move the clamps in for specified amount of time, the move them out (for another specified amount of time)
        }

        // Run the horizontal extension
        endEffector.runExtension(gamepad2.left_stick_y);
        // Run the clamps
        endEffector.runClamps();
        // Run the elevator
        elevator.runElevator(gamepad2.right_stick_y);

        // Telemetry to the driver station. NOTE: %.2f means floating point
        telemetry.addData("Robot","State (%.2f), Heading (%.2f)",
                (double) state, imu.getYaw());
        telemetry.addData("Elevator", "Velo (%.2f), Pos (%.2f), state (%.2f), goal (%.2f), power (%.2f)",
                elevator.velo, (double) elevator.getPos(), (double) elevator.getState(), (double) elevator.getGoal(), elevator.getPower());
        telemetry.addData("Clamp","State (%.2f), Power (%.2f), State time (%.2f)",
                (double) endEffector.getClampState(), endEffector.getClampPower(), endEffector.getClampStateTime());
        telemetry.addData("Drive Train","F-Left (%.2f), F-Right (%.2f), B-Left (%.2f), Back Right (%.2f)",
                driveTrain.getFrontLeftPower(), driveTrain.getFrontRightPower(), driveTrain.getBackLeftPower(), driveTrain.getBackRightPower());
    }
}