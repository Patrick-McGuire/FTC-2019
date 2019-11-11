package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;


public class EndEffector {
    private CRServo extensionSerrvo = null;
    private boolean frontLimit = false;
    private boolean rearLimit = false;

    public EndEffector(HardwareMap hardwareMap) {
        extensionSerrvo = hardwareMap.get(CRServo.class, "s1");
    }

    public void runExtension(double manualPower){
        if (Math.abs(manualPower) > .05) {
            setEtensionPower(manualPower, false); // False to account for breakage of sensors
            return;
        }
        setEtensionPower(0,false);
    }

    public void setEtensionPower(double power, boolean considerLimmits) {
        if ((!frontLimit && !rearLimit) || !considerLimmits) {
            extensionSerrvo.setPower(Range.clip(power, -1, 1));
        } else {
            extensionSerrvo.setPower(0);
        }
    }

    // Getter and setter
    public void setFrontLimit(boolean frontLimit) {
        this.frontLimit = frontLimit;
    }
    public void setRearLimit(boolean rearLimit) {
        this.rearLimit = rearLimit;
    }
    public boolean getFrontLimit() {
        return frontLimit;
    }
    public boolean getRearLimit() {
        return rearLimit;
    }
}
