package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class G9F9TeleOpHardwareMap_noIMU extends OpMode {
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;
    public DcMotor liftMotor;
    public DcMotor clawMotor;
    BNO055IMU imu;

    //debug statement for Coach Cassie
    static final boolean WHEELS = true;
    static final boolean COMP_BOT = false;
    //change to true when using comp bot

    public void init(HardwareMap hardwareMap){

if(WHEELS) {
    //get wheels
    leftFront = hardwareMap.dcMotor.get("leftFront");
    rightFront = hardwareMap.dcMotor.get("rightFront");
    leftBack = hardwareMap.dcMotor.get("leftRear");
    rightBack = hardwareMap.dcMotor.get("rightRear");
    if(COMP_BOT){
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        clawMotor = hardwareMap.dcMotor.get("clawMotor");
    }


    leftFront.setDirection(DcMotor.Direction.FORWARD);
    leftBack.setDirection(DcMotor.Direction.FORWARD);
    rightFront.setDirection(DcMotor.Direction.REVERSE);
    rightBack.setDirection(DcMotor.Direction.REVERSE);

    if(COMP_BOT) {
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        clawMotor.setDirection(DcMotor.Direction.FORWARD);

        clawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        clawMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    if(COMP_BOT) {
        clawMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //setup IMU
    //setup IMU
      /* telemetry.addLine("Init IMU :)");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode           = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit      = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit      = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        //get and initialize IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);*/

}

    }


    public void goForward(double power) {
        if (WHEELS){
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
        }
    }

    public void goBackward(double power){
        goForward(-power);
    }

    public void turnLeft(double power) {
        if (WHEELS) {
            leftFront.setPower(-power);
            rightFront.setPower(power);
            leftBack.setPower(-power);
            rightBack.setPower(power);
        }
    }
    public void turnRight(double power) {
        if (WHEELS) {
            leftFront.setPower(power);
            rightFront.setPower(-power);
            leftBack.setPower(power);
            rightBack.setPower(-power);
        }
    }

    public void strafeLeft(double power) {
        if (WHEELS) {
            leftFront.setPower(-power);
            rightFront.setPower(power);
            leftBack.setPower(power);
            rightBack.setPower(-power);
        }
    }

    public void strafeRight(double power) {
        if (WHEELS) {
            leftFront.setPower(power);
            rightFront.setPower(-power);
            leftBack.setPower(-power);
            rightBack.setPower(power);
        }
    }

    public void stopMotors() {
        if (WHEELS) {
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
        }
    }
}
