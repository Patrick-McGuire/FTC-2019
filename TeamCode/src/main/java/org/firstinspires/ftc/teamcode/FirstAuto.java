package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystems.BasicMecanum;
import org.firstinspires.ftc.teamcode.Subsystems.Clamps;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.IMU;
import org.firstinspires.ftc.teamcode.Subsystems.Vision;
import org.firstinspires.ftc.teamcode.Utility.PID;

@Autonomous(name="First Auto", group="Linear Opmode")
@Disabled
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
    boolean closedLoop = true;
    double turnPower = 0;

    private final double zOffset = 200; // Native units
    private final double xOffset = 0;   // Native units
    private final double kpXMovement = .01;
    private final double kpYMovement = -.0035;

    @Override
    public void runOpMode() {
        imu = new IMU(hardwareMap);
        elevator = new Elevator(hardwareMap);
        driveTrain = new BasicMecanum(imu, hardwareMap);
        clamps = new Clamps(hardwareMap);
        vision = new Vision(hardwareMap);
        movePID = new PID(.15,0,.05);
        clamps.setState(clamps.runToPos);
        waitForStart();
        runtime.reset();
        runtime2.reset();
        while (opModeIsActive()) {

            double xPos = vision.getxPos();
            double yPos = vision.getyPos();
            double zPos = vision.getzPos();

            if (leg == 0) {
              clamps.openArm();
              moveGoal = -950;
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
                } else if (runtime2.milliseconds() < 1500) {
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
                if ((Math.abs(zPos) - zOffset) < 40 && Math.abs(xPos) < 4 && vision.getInView() && runtime.milliseconds() > 1000) {

                    leg = 2;
                    xPower = 0;
                    yPower = 0;
                    clamps.openArm();
                    clamps.hook();
                    moveGoal = (int) driveTrain.getFrontRightPos() + 500;
                } else if (!(Math.abs(xPos) < 3 && vision.getInView())){
                    runtime.reset();

                }
            } else if (leg == 2) {
                yPower = Range.clip(movePID.runPID(moveGoal, driveTrain.getFrontRightPos()),-.3,.3);
                visActive = false;
                clamps.openArm();
                if (Math.abs(moveGoal - driveTrain.getFrontRightPos()) < 20) {
                    leg = 3;
                    yPower = 0;
                    headingGoal = -90;
                }
            } else if (leg == 3) {
                turnPower = Range.clip(imu.getYaw() - headingGoal, -.1, .1) * .001;
                closedLoop = false;
                if (Math.abs(imu.getYaw()) - 90 < 10) {
                    leg = 4;
                    moveGoal = (int) driveTrain.getFrontRightPos() - 2000;
                }
            } else if (leg == 4) {
                closedLoop = true;
                yPower = Range.clip(movePID.runPID(moveGoal, driveTrain.getFrontRightPos()),-.3,.3);
                if (Math.abs(moveGoal - driveTrain.getFrontRightPos()) < 20) {
                    leg = 5;
                    yPower = 0;
                }
            } else if (leg == 5) {
                yPower = 0;
                xPower =  0;
            }

            telemetry.addData("A",driveTrain.getFrontRightPos());
            telemetry.addData("a", leg);
            telemetry.addData("B", yPower);
            telemetry.addData("x-pos", xPos);
            telemetry.addData("y-pos", yPos);
            telemetry.addData("z-pos", zPos);
            telemetry.addData("cmp state", clamps.getState());
            telemetry.update();
            if (closedLoop) {
                driveTrain.closedLoopMecanum(xPower, yPower, headingGoal);
            }else {
                driveTrain.openLoopMecanum(0, 0, turnPower);
            }

            clamps.runClamps();
            elevator.runElevator(0);
            if (visActive) {
                vision.runVision();
            }
        }
        vision.disableLight();
    }
}