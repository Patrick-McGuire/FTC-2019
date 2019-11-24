package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="First Auto", group="Linear Opmode")
public class FirstAuto extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime runtime2 = new ElapsedTime();

    private IMU imu;
    private BasicMecanum driveTrain;
    private Elevator elevator;
    private Clamps clamps;
    private Vision vision;
    private boolean visActive = false;
    private PID movePID;
    private int moveGoal = 0;
    int leg = 0;
    double xPower = 0;
    double yPower = 0;
    double headingGoal = 0;

    private final double zOffset = 250; // Native units
    private final double xOffset = 0;   // Native units
    private final double kpXMovement = .01;
    private final double kpYMovement = -.0025;

    @Override
    public void runOpMode() {
        imu = new IMU(hardwareMap);
        elevator = new Elevator(hardwareMap);
        driveTrain = new BasicMecanum(imu, hardwareMap);
        clamps = new Clamps(hardwareMap);
        vision = new Vision(hardwareMap);
        movePID = new PID(.15,0,.05);

        waitForStart();
        runtime.reset();
        runtime2.reset();
        while (opModeIsActive()) {

            double xPos = vision.getxPos();
            double yPos = vision.getyPos();
            double zPos = vision.getzPos();

            if (leg == 0) {
              moveGoal = -750;
              yPower = Range.clip(movePID.runPID(moveGoal, driveTrain.getFrontRightPos()),-.3,.3);
              if (Math.abs(moveGoal - driveTrain.getFrontRightPos()) < 20) {
                  leg = 1;
                  yPower = 0;
              }
            } else if (leg == 1) {
                visActive = true;

                if (runtime2.milliseconds() < 1000) {
                    xPower = -.2;
                    yPower = -.075;
                } else if (runtime2.milliseconds() < 2000) {
                    xPower = 0;
                    yPower = 0;
                } else {
                    runtime2.reset();
                }

                if (vision.getInView()) {
                    xPower = (xPos - xOffset) * kpXMovement;       // Basic proportional control
                    yPower = (zPos - zOffset) * kpYMovement;      // Basic proportional control
                    xPower = Range.clip(xPower, -.2,.2);
                    yPower = Range.clip(yPower, -.2,.2);
                }
                if ((Math.abs(zPos) - zOffset) < 30 && Math.abs(xPos) < 3 && vision.getInView() && runtime.milliseconds() > 1000) {

                    leg = 2;
                    driveTrain.resetEncoder();
                    xPower = 0;
                    yPower = 0;
                } else if (!(Math.abs(xPos) < 3 && vision.getInView())){
                    runtime.reset();
                }
            } else if (leg == 2) {
                visActive = false;
                elevator.setGoal(-1200);
                moveGoal = -200;
                //yPower = Range.clip(movePID.runPID(moveGoal, driveTrain.getFrontRightPos()),-.3,.3);
            }

            telemetry.addData("A",driveTrain.frontRightDrive.getCurrentPosition());
            telemetry.addData("a", leg);
            telemetry.addData("B", yPower);
            telemetry.addData("x-pos", xPos);
            telemetry.addData("y-pos", yPos);
            telemetry.addData("z-pos", zPos);
            telemetry.addData("z-pos", runtime.milliseconds());
            telemetry.update();
            //clamps.runExtension();
            driveTrain.closedLoopMecanum(xPower, yPower, headingGoal);
            //clamps.runClamps();
            elevator.runElevator(0);
            if (visActive) {
                vision.runVision();
            }
        }
    }
}