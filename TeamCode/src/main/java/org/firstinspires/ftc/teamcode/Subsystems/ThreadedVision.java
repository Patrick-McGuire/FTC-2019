package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import com.vuforia.CameraDevice;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class ThreadedVision {
    // Init vars to keep track of data from the camera
    private double xPos = 0;
    private double yPos = 0;
    private double zPos = 0;
    private boolean inView = false;

    // Vuforia settings
    private static final String vuforiaKey = "AcMAwZX/////AAABmfcr9JCXLkUfkemo+yziQFISJO3rURaQWniX0CIlg6HFNdD4chxNZ+nByW6lMpJbuo/iaV66Mr5KF2EEqM+qg0SJaRNqKk32vHryEnHTtGhn/JBdSUJ25c+/oL9+mn7ZdaV+8EJ63bUURDcmTKzpaknYX/Cgx8i2g7v7aZP7YFdF2NqjChu13d8ZZifH2uaro2TmlC2JMhRbf+jPnOLuPyTha8GXOSuvY3nTizLJ8yn1nlXsLORCIolvYCdGhg0945koHDn4oF3XyZy1Der9nH7pkv4T/4Y/vd8YRbxfvKpdmtSPn5fEGKJ16B0bmEXOcBMnSH60BVYOaMSP7YHw8uncbnvwV0sNgjZV1OC6sgm6";
    private static final VuforiaLocalizer.CameraDirection cameraChoice = BACK;
    private static final boolean phonePORTRAIT = true;

    // Vuforia instances
    private VuforiaTrackable target;
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private VuforiaTrackables targetsSky;

    // Thread instance
    private VisionThread visionThread;
    // Constructor
    ThreadedVision(HardwareMap hardwareMap) {
        visionThread = new VisionThread();
        // Initialize vuforia stuff
        // I don't understand all of this yet
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = vuforiaKey;
        parameters.cameraDirection = cameraChoice;
        parameters.useExtendedTracking = false;         // NECESSARY: This prevents the robot from going crazy

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        // Yay done with that

        // Save these to be accessible by other functions
        target = stoneTarget;
        targetsSky = targetsSkyStone;

        // Start looking for sky stones
        targetsSkyStone.activate();
        enableLight();
    }

    private class VisionThread extends Thread {
        private double xPos = 0;
        private double yPos = 0;
        private double zPos = 0;
        private boolean inView = false;
        private boolean runThread = true;

        @Override
        public void run() {
            enableLight();
            enableTracking();
            while (runThread) {
                // Check if the target is visible
                if (((VuforiaTrackableDefaultListener) target.getListener()).isVisible()) {
                    // Update tracking info?? not sure what this does
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) target.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }

                    // Get our distance from the target
                    VectorF translation = lastLocation.getTranslation();
                    xPos = translation.get(0);
                    yPos = translation.get(1);
                    zPos = translation.get(2);
                    inView = true;
                } else {
                    // Reset the variables
                    inView = false;
                    xPos = 0;
                    yPos = 0;
                    zPos = 0;
                }
            }
        }
        // Getter methods
        public double getxPos() {
            return xPos;
        }
        public double getyPos() {
            return yPos;
        }
        public double getzPos() {
            return zPos;
        }
        public boolean getInView() {
            return inView;
        }
        public void disableThread() {
            runThread = false;
        }
    }

    // Call this in a loop to get position data
    public void runVision() {
        xPos = visionThread.getxPos();
        yPos = visionThread.getyPos();
        zPos = visionThread.getzPos();
        inView = visionThread.getInView();
    }

    // Enable and disable the skystone from the tracking code
    public void disableTracking() {
        targetsSky.deactivate();
    }

    public void enableTracking() {
        targetsSky.activate();
    }

    public void startTracking() {
        visionThread.start();
    }

    public void endTracking() {
        visionThread.disableThread();
    }

    public void resetTracking() { // Never call in a loop
        disableTracking();
        enableTracking();
    }

    public void enableLight() {
        CameraDevice.getInstance().setFlashTorchMode(true);
    }

    public void disableLight() {
        CameraDevice.getInstance().setFlashTorchMode(false);
    }

    // Getter methods
    public double getxPos() {
        return xPos;
    }

    public double getyPos() {
        return yPos;
    }

    public double getzPos() {
        return zPos;
    }

    public boolean getInView() {
        return inView;
    }
}