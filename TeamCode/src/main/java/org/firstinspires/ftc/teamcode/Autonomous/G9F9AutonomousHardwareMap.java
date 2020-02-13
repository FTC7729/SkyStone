package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public abstract class G9F9AutonomousHardwareMap extends LinearOpMode {
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;
    public DcMotor liftMotor;
    public DcMotor clawMotor;

    static final double CLAW_MIN_CLOSED = 0;
    static final double CLAW_MAX_OPEN = 5000;
    static final double CLAW_CLOSED_ON_SKYSTONE = 1130;
    static final double LIFT_UP_SKYSTONE = 1000;
    static final double LIFT_BOTTOM_MIN = 0;
    static final double LIFT_TOP_MAX =  6618;

    private ElapsedTime runtime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV_NEVEREST40    = 1120 ;    // eg: NEVEREST 40 Motor Encoder https://www.servocity.com/neverest-40-gearmotor
    static final double     COUNTS_PER_MOTOR_REV_NEVEREST20    = 560 ;
    static final double     ROTATIONS_PER_MINUTE    = 160 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    //MUST BE REMEASURED BEFORE USE. DELETE TELEMETRY AND STUFF ONCE FIXED
    static final double     WHEEL_DIAMETER_INCHES   = 4 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV_NEVEREST20
            * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    //Threshold for Gyro turning so that we will not continuously attempt to reach an exact value
    static final double THRESHOLD = 1;
    BNO055IMU imu;

    public void init(HardwareMap hardwareMap){
        //get wheels
        leftFront  = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftRear");
        rightBack  = hardwareMap.dcMotor.get("rightRear");
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        clawMotor  = hardwareMap.dcMotor.get("clawMotor");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        clawMotor.setDirection(DcMotor.Direction.FORWARD);

        clawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        clawMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        clawMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
    public void gyroTurn (double power, double target)
    {
        Orientation angles;
        double error;
        double k = 55/3600.0;
        double kInt = 3/3600.0;
        double eInt = 0;
        double startTime = System.currentTimeMillis();
        double prevTime = System.currentTimeMillis();
        while(opModeIsActive()) {
            double currentTime = System.currentTimeMillis();
            double loopTime = (currentTime - prevTime)/1000.0; // In seconds
            prevTime = currentTime;
            double timeElapsed = currentTime - startTime;
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            error = target - angles.firstAngle;
            eInt += loopTime * error;
            telemetry.addData("Heading",angles.firstAngle+" degrees");
            telemetry.addData("Loop time: ",loopTime+" ms");
            telemetry.update();
            if (error == 0 || timeElapsed/1000 >= 5){
                stopMotors();
                break;
            }
            turnLeft(k * error + kInt * eInt);
            idle();
        }
    }

    public void goForward(double power) {
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
    }

    public void goBackward(double power){
        goForward(-power);
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

    public void strafeLeft(double power) {
        leftFront.setPower(-power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(-power);
    }

    public void strafeRight(double power) {
        leftFront.setPower(power);
        rightFront.setPower(-power);
        leftBack.setPower(-power);
        rightBack.setPower(power);
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

}
