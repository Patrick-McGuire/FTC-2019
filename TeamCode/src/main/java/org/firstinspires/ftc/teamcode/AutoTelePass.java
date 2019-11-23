package org.firstinspires.ftc.teamcode;

public class AutoTelePass {
    private static int elevatorEncoder = -1;
    private static int intakePos = -1;
    private static int heading = 1000;

    // Elevator methods
    public int getElevatorEncoder() {
        return elevatorEncoder;
    }
    public void setElevatorEncoder(int elevatorEncoder) {
        this.elevatorEncoder = elevatorEncoder;
    }

    // Intake methods
    public int getIntakePos() {
        return intakePos;
    }
    public void setIntakePos(int intakePos) {
        this.intakePos = intakePos;
    }

    // Heading methods
    public int getHeading() {
        return heading;
    }
    public void setHeading(int heading) {
        this.heading = heading;
    }
}
