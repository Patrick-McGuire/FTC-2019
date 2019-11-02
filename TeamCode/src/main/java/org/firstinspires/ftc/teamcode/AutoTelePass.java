package org.firstinspires.ftc.teamcode;

public class AutoTelePass {
    private static int elevatorEncoder = -1;
    private static int intakePos = -1;
    private static int heading = 1000;

    // Elevator methods
    public int getElevatorEncoder() {
        return elevatorEncoder;
    }
    public void setElevatorEncoder(int ELEVATORENCODER) {
        elevatorEncoder = ELEVATORENCODER;
    }

    // Intake methods
    public int getIntakePos() {
        return intakePos;
    }
    public void setIntakePos(int INTAKEPOS) {
        intakePos = INTAKEPOS;
    }

    // Heading methods
    public int getHeading() {
        return elevatorEncoder;
    }
    public void setHeading(int HEADING) {
        elevatorEncoder = HEADING;
    }
}
