package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//import org.firstinspires.ftc.teamcode.subsystems.IMU;

/**
 * In case everything fails...
 */
@SuppressWarnings("unused")
@TeleOp(name = "Simple Mecanum", group = "Opmode")
public final class Teleop extends OpMode {
    private IMU imu;
    private BasicMechanum driveTrain;
    double headingGoal = 0;
    int refTime = 0;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public final void init() {
        imu = new IMU(hardwareMap);
        driveTrain = new BasicMechanum(imu, hardwareMap);
        //imu = IMU.getInstanceInit(hardwareMap);
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

        telemetry.addData("Heading", imu.getYaw());
    }
}