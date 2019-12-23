package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.IMU;
import org.firstinspires.ftc.teamcode.Subsystems.ScaledMecanum;

// Two controller teleop mode
@TeleOp(name = "Teleop", group = "Opmode")
public final class Teleop2 extends OpMode {
    // Declare OpMode members.
    private IMU imu;
    private ScaledMecanum drivetrain;
    private DcMotor ele;
    private Servo servo;


    @Override
    public final void init() {
        // Init subsystems
        imu = new IMU(hardwareMap);
        drivetrain = new ScaledMecanum(imu, hardwareMap, gamepad1, gamepad2);
        ele = hardwareMap.get(DcMotor.class, "ele");
        servo = hardwareMap.get(Servo.class, "s");
        drivetrain.enableYeet();
    }

    @Override
    public final void loop() {
        ele.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
        drivetrain.runMecanum();
        if (gamepad2.dpad_up) {
            servo.setPosition(.6);
        } else if (gamepad2.dpad_down) {
            servo.setPosition(1);
        }
    }
}