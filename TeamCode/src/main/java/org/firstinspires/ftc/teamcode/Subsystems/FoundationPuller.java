package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FoundationPuller {
    private Servo leftPuller;
    private Servo rightPuller;

    public FoundationPuller(HardwareMap hardwareMap) {
        leftPuller = hardwareMap.get(Servo.class, "s11");
        rightPuller = hardwareMap.get(Servo.class, "s12");
    }

    public void setServoPos(double pos) {
        leftPuller.setPosition(pos);
        rightPuller.setPosition(1 - pos);
    }

    public void openPuller() {
        setServoPos(.75);
    }

    public void clampPuller() {
        setServoPos(1);
    }

    public void prePuller() {
        setServoPos(.5);
    }

    public void doHankTank() {
        setServoPos(.65);
    }
}
