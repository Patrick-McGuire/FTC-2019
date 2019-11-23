package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Landing", group="Linear Opmode")
public class FirstAuto extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private IMU imu;
    private BasicMecanum driveTrain;
    private Elevator elevator;
    private EndEffector endEffector;
    private Vision vision;
    private boolean visActive = false;
    private PID movePID;
    private int moveGoal = 0;
    int leg = 0;
    double xPower = 0;
    double yPower = 0;
    double headingGoal = 0;

    @Override
    public void runOpMode() {
        imu = new IMU(hardwareMap);
        elevator = new Elevator(hardwareMap);
        driveTrain = new BasicMecanum(imu, hardwareMap);
        endEffector = new EndEffector(hardwareMap);
        vision = new Vision(hardwareMap);
        movePID = new PID(.15,0,0);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {

            if (leg == 0) {
              moveGoal = -100;
              yPower = Range.clip(movePID.runPID(moveGoal, driveTrain.getFrontRightPos()),-.5,.5);
              if (Math.abs(moveGoal - driveTrain.getFrontRightPos()) < 20) {
                  leg = 1;
                  yPower = 0;
              }
            } else if (leg == 1) {
                visActive = true;
                xPower = -.2;
                yPower = -.1;
                if (vision.getInView()) {
                    xPower = vision.getyPos() * -.1;
                    yPower = (vision.getxPos() - 4) * .1;
                    xPower = Range.clip(xPower, -.2,.2);
                    yPower = Range.clip(yPower, -.2,.2);
                }
                if (Math.abs(vision.getyPos()) < .1 && vision.getInView() && runtime.milliseconds() > 5000) {

                    leg = 2;
                    xPower = 0;
                    yPower = 0;
                } else {
                    runtime.reset();
                }
            } else if (leg == 2) {
                elevator.setGoal(500);
            }

            telemetry.addData("A",driveTrain.frontRightDrive.getCurrentPosition());
            telemetry.addData("a", leg);
            telemetry.addData("B", yPower);
            telemetry.update();
            //endEffector.runExtension();
            driveTrain.closedLoopMecanum(xPower, yPower, headingGoal);
            endEffector.runClamps();
            elevator.runElevator(0);
            if (visActive) {
                vision.runVision();
            }
        }
    }
}