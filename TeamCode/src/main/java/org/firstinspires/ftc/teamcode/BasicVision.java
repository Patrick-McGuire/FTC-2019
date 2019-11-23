package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

// This op-mode will drive until the robot is centered on a skystone, and a few inches away from it (300 native units)
@Autonomous(name="Basic Vision Example", group="Linear Opmode")
public class BasicVision extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private IMU imu;
    private BasicMecanum driveTrain;

    // Vision instance. This does everything related to tracking
    private Vision vision;

    // Init vars
    private final double zOffset = 300; // Native units
    private final double xOffset = 0;   // Native units
    private final double kpXMovement = .005;
    private final double kpYMovement = -.005;
    private double xPower = 0;
    private double yPower = 0;
    private double headingGoal = 0;

    @Override
    public void runOpMode() {
        // Display on driver station
        telemetry.addData("Initialization", "Starting");
        telemetry.update();

        // Init subsystems
        imu = new IMU(hardwareMap);
        driveTrain = new BasicMecanum(imu, hardwareMap);
        vision = new Vision(hardwareMap);;

        // Display on driver station
        telemetry.addData("Initialization", "Complete");
        telemetry.update();

        // Wait till play is pressed
        waitForStart();
        runtime.reset();

        // Tracking loop
        while (opModeIsActive()) {

            // This gets the data about location
            vision.runVision();

            // Get that^ data
            double xPos = vision.getxPos();
            double yPos = vision.getyPos();
            double zPos = vision.getzPos();

            // If there is data / not perfectly centered calculate desired motor power values
            if (xPos != 0 || yPos != 0){
                xPower = (xPos - xOffset) * kpXMovement;       // Basic proportional control
                yPower = (zPos - zOffset) * kpYMovement;      // Basic proportional control
            } else {
                // Turn of the motors if perfectly centered, or if there is no data
                xPower = 0;
                yPower = 0;
            }

            // Run the drivetrain at the desired power values
            driveTrain.closedLoopMecanum(xPower, yPower, headingGoal);

            // Display data on the driver station
            telemetry.addData("x-pos", xPos);
            telemetry.addData("y-pos", yPos);
            telemetry.addData("z-pos", zPos);
            telemetry.addData("x-power", xPower);
            telemetry.addData("y-power", yPower);
            telemetry.update();
        }
    }
}