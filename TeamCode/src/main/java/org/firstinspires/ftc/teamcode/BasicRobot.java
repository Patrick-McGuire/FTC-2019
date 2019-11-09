package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;


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
    private CRServo servo1 = null;
    @Override
    public final void init() {
        //eleMotor = hardwareMap.get(DcMotor.class, "ELE");
        imu = new IMU(hardwareMap);
        elevator = new Elevator(hardwareMap);
        driveTrain = new BasicMechanum(imu, hardwareMap);
        servo1 = hardwareMap.get(CRServo.class, "s1");

    }


    @Override
    public final void loop() {
        // Drive train code
        if(runtime.milliseconds() < 300) {
            headingGoal = imu.getYaw();
        }
        if(gamepad1.right_stick_x > .05 || gamepad1.right_stick_x < -.05) {
            driveTrain.openLoopMechanum(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            runtime.reset();
        } else {
            driveTrain.closedLoopMechanum(gamepad1.left_stick_x, gamepad1.left_stick_y, headingGoal);
        }

        servo1.setPower(gamepad2.left_stick_y);

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
            elevator.setState(11);
        }

        elevator.runElevator(gamepad2.right_stick_y);

        telemetry.addData("Velo", elevator.velo);
        telemetry.addData("Ele Pos", elevator.getPos());
        telemetry.addData("Ele state", elevator.getState());
    }
}