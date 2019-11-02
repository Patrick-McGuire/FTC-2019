package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "Teleop", group = "Opmode")
public final class Teleop extends OpMode {
    private IMU imu;

    //Subsystems
    private BasicMechanum driveTrain;
    private Intake intake;
    private Elevator elevator;

    double headingGoal = 0;

    private int elevatorGoal = 0;
    private int lastElevatorGoal = 0;
    private int stoneHeight = 1000;
    private int elevatorPlacingOffset = -200;


    private ElapsedTime runtime = new ElapsedTime();

    // State 0 is reserved for IDLE
    // Intake states: 1 - 10
    // Elevator states: 11 - 20
    // Robot States: 21 - 30
    private int state = 0;
    private int idle = 0;
    // Intake States
    private int running = 1;
    private int opening = 2;
    private int open = 3;
    // Elevator states
    private int zeroing = 11;
    private int moving = 12;
    // Robot states
    private int intaking = 21;
    private int handingOff = 22;
    private int holdingStone = 23;
    private int placing = 24;

    @Override
    public final void init() {
        imu = new IMU(hardwareMap);

        driveTrain = new BasicMechanum(imu, hardwareMap);
        intake = new Intake(hardwareMap);
        elevator = new Elevator(hardwareMap);

        runtime.reset();
    }


    @Override
    public final void loop() {
        if(runtime.milliseconds() < 300) {
            headingGoal = imu.getYaw();
        }
        if(gamepad1.right_stick_x > .05 || gamepad1.right_stick_x < -.05) {
            driveTrain.openLoopMechanum(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            runtime.reset();
        } else {
            driveTrain.closedLoopMechanum(gamepad1.left_stick_x, gamepad1.left_stick_y, headingGoal);
        }

        // Operator inputs
        if (gamepad2.x) {
            elevatorGoal = 0;
        } else if (gamepad2.a) {
            elevatorGoal = lastElevatorGoal - stoneHeight;
            lastElevatorGoal = elevatorGoal;
        }  else if (gamepad2.b) {
            elevatorGoal = lastElevatorGoal;
        }  else if (gamepad2.y) {
            elevatorGoal = lastElevatorGoal + stoneHeight;
            lastElevatorGoal = elevatorGoal;
        }  else if (gamepad2.dpad_down) {
            elevator.setState(zeroing);
        }

        // Driver control inputs
        if (gamepad1.right_bumper) {
            state = intaking;
        } else if (gamepad1.left_bumper) {
            state = idle;
        } else if (gamepad1.a) {
            state = handingOff;
        } else if (gamepad1.right_trigger > .05) {
            if (gamepad1.right_trigger > .5) {
                // Relese game peice
                state = idle;
            } else {
                state = placing;
            }
        } else if (state == placing) {
            state = holdingStone;
        }

        // State cases
        if (state == idle) {
            intake.runIntake(true, false);
            elevator.runElevator(elevatorGoal, gamepad2.right_stick_x);
        } else if (state == intaking) {
            intake.runIntake(false,true);
            elevator.runElevator(0, gamepad2.right_stick_x);
        } else if (state == handingOff) {
            intake.runIntake(false,false);
        } else if (state == holdingStone) {
            intake.runIntake(true,false);
            elevator.runElevator(elevatorGoal, gamepad2.right_stick_x);
        } else if (state == placing) {
            intake.runIntake(true,false);
            elevator.runElevator(elevatorGoal - elevatorPlacingOffset, gamepad2.right_stick_x);
        }

        int intakeState = intake.getState();
        int elevatorState = elevator.getState();
        int elevatorPos = elevator.getPos();
        telemetry.addData("Heading", imu.getYaw());
    }
}