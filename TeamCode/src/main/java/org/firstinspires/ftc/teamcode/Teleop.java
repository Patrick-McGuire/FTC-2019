package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Teleop 2C", group = "Opmode")
public final class Teleop extends OpMode {
    private IMU imu;
    private BasicMechanum driveTrain;
    double headingGoal = 0;
    boolean pressLastTime = false;
    private Elevator elevator;
    private ElapsedTime runtime = new ElapsedTime();
    private EndEffector endEffector;

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
        imu = new IMU(hardwareMap);
        elevator = new Elevator(hardwareMap);
        driveTrain = new BasicMechanum(imu, hardwareMap);
        endEffector = new EndEffector(hardwareMap);
    }


    @Override
    public final void loop() {
        double gamepadX = gamepad1.left_stick_x;
        double gamepadY = gamepad1.left_stick_y;
        double gamepadTurn = gamepad1.right_stick_x;

        double xPower = -Math.pow(gamepadX, 2) * (Math.abs(gamepadX) / (gamepadX + .0001));
        double yPower = Math.pow(gamepadY, 2) * (Math.abs(gamepadY) / (gamepadY + .0001));
        double turnPower = Math.pow(gamepadTurn, 2) * (Math.abs(gamepadTurn) / (gamepadTurn + .0001));

        // Drive train code
        if(runtime.milliseconds() < 300) {
            headingGoal = imu.getYaw();
        }
        if(Math.abs(imu.getYaw() - headingGoal) > 30) {
            headingGoal = imu.getYaw();
        }
        if(gamepad1.right_stick_x > .05 || gamepad1.right_stick_x < -.05) {
            driveTrain.openLoopMechanum(xPower, yPower, turnPower);
            runtime.reset();
        } else {
            driveTrain.closedLoopMechanum(xPower, yPower, headingGoal);
        }

        endEffector.runExtension(gamepad2.left_stick_x);

        // Elevator code
        boolean gp2x = gamepad2.x;
        boolean gp2a = gamepad2.a;
        boolean gp2b = gamepad2.b;
        boolean gp2y = gamepad2.y;
        if (gp2x && !pressLastTime){
            elevator.setState(quickZero);
            endEffector.setClampState(zeroingClamp);
            pressLastTime = true;
        } else if (gp2a && !pressLastTime){
            elevator.goalDownBlock();
            pressLastTime = true;
        } else if (gp2b && !pressLastTime) {
            elevator.runGoal();
            pressLastTime = true;
        } else if (gp2y && !pressLastTime) {
            elevator.goalUpBlock();
            pressLastTime = true;
        } else if (gp2x || gp2a || gp2b || gp2y) {
            pressLastTime = true;
        } else {
            pressLastTime = false;
        }
        if (gamepad2.dpad_down) {
            elevator.setState(zeroing);
        }
        if (gamepad1.right_trigger > .05) {
            if (gamepad1.right_trigger < .8) {
                elevator.place();
            } else if (gamepad1.right_trigger >= .8){
                elevator.delayedMove(elevator.getGoal(), 3000);
                endEffector.setClampState(releasingClamp);
            }
        } else if (elevator.getState() != delayedState){
            elevator.release();
        }

        if (gamepad1.right_bumper) {
            endEffector.setClampState(clamping);
        } else if(gamepad1.left_bumper) {
            endEffector.setClampState(releasingClamp);
        } else if (gamepad1.x) {
            endEffector.setClampState(zeroingClamp);
        }
        endEffector.runClamps();
        elevator.runElevator(gamepad2.right_stick_y);

        // Telemetry to the driver station
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