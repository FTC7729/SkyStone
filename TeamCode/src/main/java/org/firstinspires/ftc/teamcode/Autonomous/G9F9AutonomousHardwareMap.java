package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.mmPerInch;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


public abstract class G9F9AutonomousHardwareMap extends LinearOpMode {
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;
    public DcMotor liftMotor;
    public DcMotor clawMotor;

    public DistanceSensor rightDistanceSensor;
    public DistanceSensor leftDistanceSensor;
		
		/* Encoder Values for Claw + Lift */
		static final int CLAW_MIN_CLOSED = 0;
    static final int CLAW_MAX_OPEN = 5000;
    static final int CLAW_CLOSED_ON_SKYSTONE = 1350;
    static final int LIFT_UP_SKYSTONE = 1000;
    static final int LIFT_BOTTOM_MIN = 0;
    static final int LIFT_TOP_MAX = 6618;

    private ElapsedTime runtime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV_NEVEREST40    = 1120 ;    // eg: NEVEREST 40 Motor Encoder https://www.servocity.com/neverest-40-gearmotor
    static final double     COUNTS_PER_MOTOR_REV_NEVEREST20    = 540 ;
    static final double     ROTATIONS_PER_MINUTE    = 160 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    //MUST BE REMEASURED BEFORE USE. DELETE TELEMETRY AND STUFF ONCE FIXED
    static final double     WHEEL_DIAMETER_INCHES   = 4 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV_NEVEREST20
            * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    //Threshold for Gyro turning so that we will not continuously attempt to reach an exact value
    static final double THRESHOLD = 3;
    BNO055IMU imu;

    public void init(HardwareMap hardwareMap){
        //get wheels
        leftFront  = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftRear");
        rightBack  = hardwareMap.dcMotor.get("rightRear");
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        clawMotor  = hardwareMap.dcMotor.get("clawMotor");

        rightDistanceSensor  = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");
        leftDistanceSensor  = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");



        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        clawMotor.setDirection(DcMotor.Direction.FORWARD);

        clawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        clawMotor.setTargetPosition(0);
        clawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        clawMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //setup IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode           = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit      = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit      = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        //get and initialize IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
    }
    boolean targetVisible = false;

    List<VuforiaTrackable> allTrackables;
    VuforiaTrackables targetsSkyStone;

    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia = null;
    public void initvuforia(){
        final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
         final boolean PHONE_IS_PORTRAIT = false  ;
        //experimental thing here, may frick stuff up
        //public int msStuckDetectInit     = 8000;
        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code on the next line, between the double quotes.
         */
         final String VUFORIA_KEY =
                "Aa0ODYT/////AAABmbrSV1REZUlljIiCpPNPvdlR2T8o80Rzjt2HYPF1L/yMqgYtiPiS2wISQ/Hl5yWPn8/BGo9Gxj4Ik583p3trh7q4Yaw/l4F0+MhN6ApKPq6vYImunRyDouiULcV+Bd/haLM+G754/3Srfw11SdWKJjFfjYHafrNOkTqEZhyCQAza2xSWKHKtAMZot93WU6YM4wXlCrosFHWc4/YQZJb0BPi6m7R/7dJSVm8PR7jUfT8mnlW0A+q0K151xQtfxbj0CMGqIilLihCP1x8rYaWiAAaCwJU3slDaR22x9DqpRQ6E8+IryNxrZW2kX14rzCEax2EdnsrNi3SXdswXS+LsH3UfjtxU/sxLtjmFA8ekk0sQ";

        // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
        // We will define some constants and conversions here
         final float mmPerInch        = 25.4f;
         final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

        // Constant for Stone Target
         final float stoneZ = 2.00f * mmPerInch;

        // Constants for the center support targets
         final float bridgeZ = 6.42f * mmPerInch;
         final float bridgeY = 23 * mmPerInch;
         final float bridgeX = 5.18f * mmPerInch;
         final float bridgeRotY = 59;                                 // Units are degrees
         final float bridgeRotZ = 180;

        // Constants for perimeter targets
         final float halfField = 72 * mmPerInch;
         final float quadField  = 36 * mmPerInch;

        // Class Members
         OpenGLMatrix lastLocation = null;
         VuforiaLocalizer vuforia = null;

        /**
         * This is the webcam we are to use. As with other hardware devices such as motors and
         * servos, this device is identified using the robot configuration tool in the FTC application.
         */
        WebcamName webcamName = null;

         //boolean targetVisible = false;
         float phoneXRotate    = 0;
         float phoneYRotate    = 0;
         float phoneZRotate    = 0;

        List<VuforiaTrackable> allTrackables;
        VuforiaTrackables targetsSkyStone;

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.addWebcamCalibrationFile("teamwebcamcalibrations.xml");
        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        telemetry.addData("Parameter stuff!","");
        telemetry.update();

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        targetsSkyStone = vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");
        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        telemetry.addData("Perimeter stuff!","");
        telemetry.update();

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        telemetry.addData("Set location of pictures!","");
        telemetry.update();
        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 6.25f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 6.45f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 4.25f * mmPerInch;     // eg: Camera is ON the robot's center line
        telemetry.addData("Rotated Cameraz!","");
        telemetry.update();

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:

        // waitForStart();

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.

        targetsSkyStone.activate();
    }
    public void updatevuforia (){
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        }
        else {
            telemetry.addData("Visible Target", "none");
        }
        telemetry.update();




    }
    public void gyroTurn (double power, double target)
    {
        Orientation angles;
        double error;
        double k = 55/3600.0;
        double kInt = 3/3600.0;
        double eInt = 0;
        double startTime = System.currentTimeMillis();
        double prevTime = System.currentTimeMillis();
        double globalAngle = 0;
        double lastAngle = 0;
        double deltaAngle = 0;
        while(opModeIsActive()) {
            double currentTime = System.currentTimeMillis();
            double loopTime = (currentTime - prevTime)/1000.0; // In seconds
            prevTime = currentTime;
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //finds the angle given by the imu [-180, 180]
            double angle = angles.firstAngle;
            deltaAngle = angle - lastAngle;

            //adjusts the change in angle (deltaAngle) to be the actual change in angle
            if (deltaAngle < -180){
                deltaAngle += 360;
            }
            else if (deltaAngle > 180){
                deltaAngle -= 360;
            }
            globalAngle += deltaAngle;
            lastAngle = angle;

            error = target - globalAngle;
            eInt += loopTime * error;
            if (Math.abs(error) < THRESHOLD){
                eInt = 0;
            }
            telemetry.addData("Heading",angles.firstAngle+" degrees");
            telemetry.addData("Loop time: ",loopTime+" ms");
            telemetry.update();
            if (error == 0){
                stopMotors();
                break;
            }
            turnLeft(k * error + kInt * eInt);
            idle();
        }
        stopMotors();
    }

    public void goForward(double power, int distance) {
        Orientation angles;
        double startAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double error;
        double k = 3/360.0;
        int leftFrontTarget = leftFront.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        int rightFrontTarget = rightFront.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        int leftBackTarget = leftBack.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        int rightBackTarget = rightBack.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);

        while (opModeIsActive() &&
                (power > 0 && leftFront.getCurrentPosition() < leftFrontTarget && rightFront.getCurrentPosition() < rightFrontTarget && leftBack.getCurrentPosition() < leftBackTarget && rightBack.getCurrentPosition() < rightBackTarget) ||
                (power < 0 && leftFront.getCurrentPosition() > leftFrontTarget && rightFront.getCurrentPosition() > rightFrontTarget && leftBack.getCurrentPosition() > leftBackTarget && rightBack.getCurrentPosition() > rightBackTarget)
        ) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //finds the angle given by the imu [-180, 180]
            double angle = angles.firstAngle;
            error = startAngle - angle;
            telemetry.addData("firstAngle",angles.firstAngle+" degrees");
            telemetry.addData("leftFront ",leftFront.getCurrentPosition());
            telemetry.addData("rightFront ",rightFront.getCurrentPosition());
            telemetry.addData("leftBack ",leftBack.getCurrentPosition());
            telemetry.addData("rightBack ",rightBack.getCurrentPosition());

            telemetry.update();
            leftFront.setPower((power - (error * k)));
            rightFront.setPower((power + (error * k)));
            leftBack.setPower((power - (error * k)));
            rightBack.setPower((power + (error * k)));
        }
        stopMotors();

    }

    public void goBackward(double power, int distance){
        goForward(-power, -distance);
    }

    public void turnLeft(double power) {
        leftFront.setPower(-power);
        rightFront.setPower(power);
        leftBack.setPower(-power);
        rightBack.setPower(power);
    }
    public void turnRight(double power) {
        leftFront.setPower(power);
        rightFront.setPower(-power);
        leftBack.setPower(power);
        rightBack.setPower(-power);
    }

    public void strafeLeft(double power, int distance) {
        Orientation angles;
        double error;
        double k = 3/360.0;
        double startAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        int leftFrontTarget = leftFront.getCurrentPosition() - (int)(distance * COUNTS_PER_INCH);
        int rightFrontTarget = rightFront.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        int leftBackTarget = leftBack.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        int rightBackTarget = rightBack.getCurrentPosition() - (int)(distance * COUNTS_PER_INCH);
        leftFront.setTargetPosition(leftFrontTarget);
        rightFront.setTargetPosition(rightFrontTarget);
        leftBack.setTargetPosition(leftBackTarget);
        rightBack.setTargetPosition(rightBackTarget);

        while (opModeIsActive()
                &&(leftFront.getCurrentPosition() > leftFrontTarget && rightFront.getCurrentPosition() < rightFrontTarget && leftBack.getCurrentPosition() < leftBackTarget && rightBack.getCurrentPosition() > rightBackTarget)) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //finds the angle given by the imu [-180, 180]
            double angle = angles.firstAngle;
            error = startAngle - angle;
            leftFront.setPower(-(power + (error * k)));
            rightFront.setPower((power + (error * k)));
            leftBack.setPower((power - (error * k)));
            rightBack.setPower(-(power - (error * k)));
            telemetry.addData("error: ",error);
            telemetry.addData("leftfront dest: ", leftFrontTarget);
            telemetry.addData("leftFront pos: ",leftFront.getCurrentPosition());


            telemetry.update();
        }
        stopMotors();
    }

    public void strafeRight(double power, int distance) {
        Orientation angles;
        double error;
        double k = 3/360.0;
        double startAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        int leftFrontTarget = leftFront.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        int rightFrontTarget = rightFront.getCurrentPosition() - (int)(distance * COUNTS_PER_INCH);
        int leftBackTarget = leftBack.getCurrentPosition() - (int)(distance * COUNTS_PER_INCH);
        int rightBackTarget = rightBack.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        leftFront.setTargetPosition(leftFrontTarget);
        rightFront.setTargetPosition(rightFrontTarget);
        leftBack.setTargetPosition(leftBackTarget);
        rightBack.setTargetPosition(rightBackTarget);

        while (opModeIsActive()
                &&(leftFront.getCurrentPosition() < leftFrontTarget && rightFront.getCurrentPosition() > rightFrontTarget && leftBack.getCurrentPosition() > leftBackTarget && rightBack.getCurrentPosition() < rightBackTarget)) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //finds the angle given by the imu [-180, 180]
            double angle = angles.firstAngle;
            error = startAngle - angle;
            leftFront.setPower((power - (error * k)));
            rightFront.setPower(-(power - (error * k)));
            leftBack.setPower(-(power + (error * k)));
            rightBack.setPower((power + (error * k)));
            telemetry.addData("error: ",error);
            telemetry.addData("leftfront dest: ", leftFrontTarget);
            telemetry.addData("leftFront pos: ",leftFront.getCurrentPosition());


            telemetry.update();
        }
        stopMotors();
    }

    public void stopMotors() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    public void strafeRightEncoder(double power, double distance, double timeout)
    {
        encoderDrive(power,distance,-distance,-distance,distance,timeout);
    }

    public void strafeLeftEncoder(double power, double distance, double timeout)
    {
        encoderDrive(power,-distance, distance, distance, -distance, timeout);
    }

    public void encoderDrive(double speed, double leftInches, double rightInches, double leftBackInches, double rightBackInches, double timeoutS) {
        /*
        telemetry.addData("CAUTION", "YOU'RE ILLEGAL, ENSURE THAT THE WHEEL HAS BEEN REMEASURED");
        boolean quit = true;
        if (quit)
        {
            return;
        }
        */
        int newLeftTarget;
        int newRightTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        if (opModeIsActive()) {
            newLeftTarget = leftFront.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightFront.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftBackTarget = leftBack.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
            newRightBackTarget = rightBack.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);
            leftFront.setTargetPosition(newLeftTarget);
            rightFront.setTargetPosition(newRightTarget);
            leftBack.setTargetPosition(newLeftBackTarget);
            rightBack.setTargetPosition(newRightBackTarget);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFront.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            leftBack.setPower(Math.abs(speed));
            rightBack.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Going to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Currently at %7d :%7d",
                        leftFront.getCurrentPosition(),
                        rightFront.getCurrentPosition(),
                        leftBack.getCurrentPosition(),
                        rightBack.getCurrentPosition()
                );
                telemetry.update();
            }

            // Stop all motion;
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


    }
    public void setClawPosition(int pos, double speed) {
        clawMotor.setTargetPosition(pos);
        clawMotor.setPower(speed);
        while ((clawMotor.getCurrentPosition() > clawMotor.getTargetPosition() + 10 || clawMotor.getCurrentPosition() < clawMotor.getTargetPosition() - 10) && opModeIsActive()) {
            telemetry.addData("Encoder Position", clawMotor.getCurrentPosition());
            telemetry.update();
            idle();
        }
        clawMotor.setPower(0);

    }
    public void setLiftPosition(int pos, double speed) {
        liftMotor.setTargetPosition(pos);
        liftMotor.setPower(speed);
        while ((liftMotor.getCurrentPosition() > liftMotor.getTargetPosition() + 10 || liftMotor.getCurrentPosition() < liftMotor.getTargetPosition() - 10) && opModeIsActive()) {
            telemetry.addData("Encoder Position", liftMotor.getCurrentPosition());
            telemetry.update();
            //idle();
        }
        liftMotor.setPower(0);
    }

}
