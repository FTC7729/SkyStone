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
    static final int CLAW_CLOSED_ON_SKYSTONE = 2000;
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
