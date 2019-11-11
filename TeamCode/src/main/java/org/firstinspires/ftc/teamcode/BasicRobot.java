package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "BasicRobot", group = "Opmode")
public final class BasicRobot extends OpMode {
    private IMU imu;

    //Telemetrey print;
    private BasicMechanum driveTrain;
    private DcMotor eleMotor;
    double headingGoal = 0;
    boolean pressLastTime = false;
    private Elevator elevator;
    private ElapsedTime runtime = new ElapsedTime();
    private EndEffector endEffector;
    private CRServo servo1 = null;
    private CRServo servo2 = null;

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
        //eleMotor = hardwareMap.get(DcMotor.class, "ELE");
        imu = new IMU(hardwareMap);
        elevator = new Elevator(hardwareMap);
        driveTrain = new BasicMechanum(imu, hardwareMap);
        endEffector = new EndEffector(hardwareMap);
        servo1 = hardwareMap.get(CRServo.class, "s2");
        servo2 = hardwareMap.get(CRServo.class, "s3");
    }


    @Override
    public final void loop() {
        // Drive train code
        if(runtime.milliseconds() < 300) {
            headingGoal = imu.getYaw();
        }
        if(Math.abs(imu.getYaw() - headingGoal) > 30) {
            headingGoal = imu.getYaw();
        }
        if(gamepad1.right_stick_x > .05 || gamepad1.right_stick_x < -.05) {
            driveTrain.openLoopMechanum(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            runtime.reset();
        } else {
            driveTrain.closedLoopMechanum(gamepad1.left_stick_x, gamepad1.left_stick_y, headingGoal);
        }

        endEffector.runExtension(gamepad2.left_stick_x);

        // Evevator code
        boolean gp2x = gamepad2.x;
        boolean gp2a = gamepad2.a;
        boolean gp2b = gamepad2.b;
        boolean gp2y = gamepad2.y;
        if (gp2x && !pressLastTime){
            elevator.setGoal(0 - elevator.zeroPos);
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
                elevator.release();
            }
        } else {
            elevator.release();
        }
        servo1.setPower(-gamepad2.left_trigger + gamepad2.right_trigger);
        servo2.setPower(gamepad2.left_trigger + -gamepad2.right_trigger);
        elevator.runElevator(gamepad2.right_stick_y);

        telemetry.addData("Velo", elevator.velo);
        telemetry.addData("Ele Pos", elevator.getPos());
        telemetry.addData("Ele state", elevator.getState());
        telemetry.addData("Ele goal", elevator.getGoal());
    }
}