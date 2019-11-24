package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Extension {
    // Declare class members
    private CRServo extensionServo;
    private ElapsedTime runtime;

    // To Keep track if the extension is at either limit
    private boolean frontLimit = false;
    private boolean rearLimit = false;


    public Extension(HardwareMap hardwareMap) {
        extensionServo = hardwareMap.get(CRServo.class, "s1");
        extensionServo.setDirection(DcMotorSimple.Direction.REVERSE);
        runtime = new ElapsedTime();
    }

    // Main method
    // Call in a loop. Runs the extension
    public void runExtension(double manualPower){
        if (Math.abs(manualPower) > .05) {
            setExtensionPower(manualPower, false);
            return;
        }
        setExtensionPower(0,false);
    }

    // Private methods
    private void setExtensionPower(double power, boolean considerLimmits) {
        if ((!frontLimit && !rearLimit) || !considerLimmits) {
            extensionServo.setPower(Range.clip(power, -1, 1));
        } else {
            extensionServo.setPower(0);
        }
    }

    // Public methods
    // Setter methods
    public void setFrontLimit(boolean frontLimit) {
        this.frontLimit = frontLimit;
    }
    public void setRearLimit(boolean rearLimit) {
        this.rearLimit = rearLimit;
    }

    // Getter methods
    public boolean getFrontLimit() {
        return frontLimit;
    }
    public boolean getRearLimit() {
        return rearLimit;
    }
}
