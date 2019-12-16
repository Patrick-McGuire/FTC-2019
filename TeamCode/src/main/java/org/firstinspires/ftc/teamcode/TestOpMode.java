package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Vision;

@Autonomous(name="Test thing", group="Linear Opmode")
@Disabled
public class TestOpMode extends LinearOpMode {

    // Declare OpMode members.
    //ThreadedVision vision;
    Vision vision;
    ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {
        vision = new Vision(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {

            vision.runVision();
            double x = vision.getxPos();
            double y = vision.getyPos();
            double z = vision.getzPos();
            boolean v = vision.getInView();
            telemetry.addData("Time", runtime.milliseconds());
            runtime.reset();
            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.addData("z", z);
            telemetry.addData("v", v);
            telemetry.update();

        }
    }
}