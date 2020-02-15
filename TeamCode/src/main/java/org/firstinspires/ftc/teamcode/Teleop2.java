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
@Disabled
public final class Teleop2 extends OpMode {
    // Declare OpMode members.
    private IMU imu;
    private ScaledMecanum drivetrain;
    private DcMotor ele;
    private DcMotor el;
    private DcMotor elee;
//    private Servo servo;


    @Override
    public final void init() {
        // Init subsystems
        imu = new IMU(hardwareMap);
        drivetrain = new ScaledMecanum(imu, hardwareMap, gamepad1, gamepad2);
        ele = hardwareMap.get(DcMotor.class, "e");
        el = hardwareMap.get(DcMotor.class, "e2");
        elee = hardwareMap.get(DcMotor.class, "e22");
//        servo = hardwareMap.get(Servo.class, "s");
        drivetrain.enableYeet();
    }

    @Override
    public final void loop() {
        elee.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        drivetrain.runMecanum();
        if (gamepad1.dpad_up) {
            ele.setPower(1);
            el.setPower(-1);
//            servo.setPosition(.6);
        } else if (gamepad1.dpad_down) {
            ele.setPower(-1);
            el.setPower(1);
//            servo.setPosition(1);
        } else {
            el.setPower(0);
            ele.setPower(0);
        }
    }
}