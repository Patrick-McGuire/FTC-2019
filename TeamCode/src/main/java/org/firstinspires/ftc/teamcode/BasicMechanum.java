package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BasicMechanum {
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    IMU imu;
    PID headingPID;

    public BasicMechanum (IMU imu, HardwareMap hardwareMap) {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "F_L");
        frontRightDrive = hardwareMap.get(DcMotor.class, "F_R");
        backLeftDrive = hardwareMap.get(DcMotor.class, "B_L");
        backRightDrive = hardwareMap.get(DcMotor.class, "B_R");
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        this.imu = imu;

        headingPID = new PID(-0.04,0,-.05);
    }

    public void openLoopMechanum(double x, double y, double yaw) {
        final double absPower = Math.hypot(x, y);
        final double driveAngle = Math.atan2(y, x) - Math.PI / 4;
        frontLeftDrive.setPower(absPower * Math.cos(driveAngle) - yaw);
        frontRightDrive.setPower(absPower * Math.sin(driveAngle) + yaw);
        backLeftDrive.setPower(absPower * Math.sin(driveAngle) - yaw);
        backRightDrive.setPower(absPower * Math.cos(driveAngle) + yaw);
    }

    public void closedLoopMechanum(double x, double y, double headingGoal) {
        double headingState = imu.getYaw();
        double yawPower = headingPID.runPID(headingGoal, headingState);
        openLoopMechanum(x, y, yawPower);
    }

    // Getter and setter
    public double getFrontLeftPower() {
        return frontLeftDrive.getPower();
    }
    public double getFrontRightPower() {
        return frontRightDrive.getPower();
    }
    public double getBackLeftPower() {
        return backLeftDrive.getPower();
    }
    public double getBackRightPower() {
        return backRightDrive.getPower();
    }
}
