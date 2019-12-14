package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Autonomous.BoxHAutonomousHardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


@Autonomous(name = "VuforiaTest", group = "Vuforia")
public class VuforiaTest extends BoxHAutonomousHardwareMap {
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;
    private static final String VUFORIA_KEY =
            "Aa0ODYT/////AAABmbrSV1REZUlljIiCpPNPvdlR2T8o80Rzjt2HYPF1L/yMqgYtiPiS2wISQ/Hl5yWPn8/BGo9Gxj4Ik583p3trh7q4Yaw/l4F0+MhN6ApKPq6vYImunRyDouiULcV+Bd/haLM+G754/3Srfw11SdWKJjFfjYHafrNOkTqEZhyCQAza2xSWKHKtAMZot93WU6YM4wXlCrosFHWc4/YQZJb0BPi6m7R/7dJSVm8PR7jUfT8mnlW0A+q0K151xQtfxbj0CMGqIilLihCP1x8rYaWiAAaCwJU3slDaR22x9DqpRQ6E8+IryNxrZW2kX14rzCEax2EdnsrNi3SXdswXS+LsH3UfjtxU/sxLtjmFA8ekk0sQ";
    //region Measurements
    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;


    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;
    //endregion Measurements

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;
    public void runOpMode() {
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.addWebcamCalibrationFile("teamwebcamcalibrations.xml");
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
    }
}