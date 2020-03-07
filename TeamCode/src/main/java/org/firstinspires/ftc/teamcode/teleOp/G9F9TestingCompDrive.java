package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Holonomic Test Drive", group="A")
public class G9F9TestingCompDrive extends G9F9TeleOpHandler {
    //lift values are starting from the bottom
    //LIFT_MAX_POS should be positive
    public final int LIFT_MAX_POS = 6618;

    public final int LIFT_MIN_POS = 0;

    public final int LIFT_ON_FOUNDATION = 770;

    public final int CLAW_MAX_POS = 5000;

    public final int CLAW_MIN_POS = 0;

    public final double ROTATION_CONSTANT = 0.4;

    double subtractAngle = 0;

    double speedScale = 1;
    double prevTime = 0;
    double startTime = System.currentTimeMillis();
    double prevLStick = 0;
    double prevMag1 = 0;
    double prevMag2 = 0;
    double timeSinceLastIncrement = 0;
    final double timeIncrement = 50;
    final double powIncrement = 0.2;
    final double deadZone = 0.05;
    public void handleGamepad1(Gamepad gamepad) {
        double rStickX;
        double rStickY;
        double lStickX;
        double targetAngle;
        double mag1;
        double mag2;
        double rotationPower;
        double maxPower;
        double scaleDown;
        double currentAngle;
        double totalAngle;
        Orientation angles;
        boolean lBumper;
        boolean rBumper;
        boolean dpadLeft;
        boolean dpadRight;
        boolean dpadUp;
        boolean dpadDown;
        boolean aPress;

        //in the beginning, timeSinceLastIncrement is increased by the current time (which is some number in the millions). this undoes that
        if (prevTime == 0){
            timeSinceLastIncrement -= System.currentTimeMillis();
        }

        //updates time
        double currentTime = System.currentTimeMillis();
        double loopTime = currentTime - prevTime;
        double timeSinceStart = currentTime - startTime;
        prevTime = currentTime;
        timeSinceLastIncrement += loopTime;

        //
        // Get the current relative heading of the robot
        //
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        currentAngle = (double) angles.firstAngle - subtractAngle;
        telemetry.addData("Heading: ", "%.3f", currentAngle);

        rStickX = -gamepad.right_stick_x * speedScale;
        rStickY = -gamepad.right_stick_y * speedScale;
        lStickX = gamepad.left_stick_x * speedScale;
        lBumper = gamepad.left_bumper;
        rBumper = gamepad.right_bumper;
        dpadLeft = gamepad.dpad_left;
        dpadRight = gamepad.dpad_right;
        dpadUp = gamepad.dpad_up;
        dpadDown = gamepad.dpad_down;
        aPress = gamepad.a;


        //checks if enough time has passed to change the power of motors
        int numOfIncrements = 0;
        //we have to increment later, so we save how often we do it now
        while (timeSinceLastIncrement > timeIncrement){
            timeSinceLastIncrement -= timeIncrement;
            //changes the power of motors
            if (lStickX > prevLStick){
                prevLStick += powIncrement;
            }
            else if (lStickX < prevLStick){
                prevLStick -= powIncrement;
            }

            numOfIncrements++;
        }
        if (Math.abs(prevLStick) < deadZone){
            prevLStick = 0;
        }
        lStickX = prevLStick;


        if (aPress){
            subtractAngle = (double) angles.firstAngle;
        }
        if (lStickX > 0) {
            lStickX = Math.pow(lStickX, 1.6);
        }else{
            lStickX = -Math.pow(-lStickX, 1.6);
        }
        targetAngle = (Math.atan2(rStickY,rStickX));
        totalAngle = targetAngle + currentAngle;
        if (lBumper){ //slow drive
            speedScale = 0.25;
        }
        if (rBumper){ //normal drive
            speedScale = 1;
        }

        if(dpadLeft){
            strafeLeft(0.5);
        } else if(dpadRight){
            strafeRight(0.5);
        } else if(dpadUp){
            goForward(0.5);
        } else if(dpadDown){
            goBackward(0.5);
        } else {


            rotationPower = -lStickX * ROTATION_CONSTANT;
            mag1 = Math.sqrt(Math.pow(rStickX, 2) + Math.pow(rStickY, 2)) * (Math.sin(totalAngle + Math.PI / 4));
            mag2 = Math.sqrt(Math.pow(rStickX, 2) + Math.pow(rStickY, 2)) * (Math.sin(totalAngle - Math.PI / 4));

            maxPower = Math.max(Math.abs(mag1) + Math.abs(rotationPower), Math.abs(mag2) + Math.abs(rotationPower));

            //adjusts mag1 and 2 so that it's smooth
            for (int i = 0; i < numOfIncrements; i++){
                if (mag1 > prevMag1){
                    prevMag1 += powIncrement;
                }
                else if (mag1 < prevMag1){
                    prevMag1 -= powIncrement;
                }

                if (mag2 > prevMag2){
                    prevMag2 += powIncrement;
                }
                else if (mag2 < prevMag2){
                    prevMag2 -= powIncrement;
                }
            }

            if (Math.abs(prevMag1) < deadZone){
                prevMag1 = 0;
            }
            if (Math.abs(prevMag2) < deadZone){
                prevMag2 = 0;
            }

            mag1 = prevMag1;
            mag2 = prevMag2;

            telemetry.addData("mag1: ", "%.3f", mag1);
            telemetry.addData("mag2: ", "%.3f", mag2);
            telemetry.addData("rotationPower: ", "%.3f", rotationPower);

            if (maxPower > 1.0)
                scaleDown = 1.0 / maxPower;
            else
                scaleDown = 1.0;


            leftFront.setPower((mag2 - rotationPower) * scaleDown);
            leftBack.setPower((mag1 - rotationPower) * scaleDown);
            rightBack.setPower((mag2 + rotationPower) * scaleDown);
            rightFront.setPower((mag1 + rotationPower) * scaleDown);
        }
    }

    public void handleGamepad2(Gamepad gamepad){
        boolean dpadUp;
        boolean dpadDown;
        boolean aPress;
        boolean bPress;
        boolean xPress;

        dpadUp = gamepad.dpad_up;
        dpadDown = gamepad.dpad_down;
        aPress = gamepad.a;
        bPress = gamepad.b;
        xPress = gamepad.x;

        if (dpadUp) {
            liftMotor.setTargetPosition(LIFT_MAX_POS);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(1);
        }
        else if (dpadDown) {
            liftMotor.setTargetPosition(LIFT_MIN_POS);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(1);
        }
        else if (xPress){
            liftMotor.setTargetPosition(LIFT_ON_FOUNDATION);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(1);
        }
        else {
            liftMotor.setPower(0);
        }

        //see above comment
        if (aPress) {
            clawMotor.setTargetPosition(CLAW_MIN_POS);
            clawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            clawMotor.setPower(3);
        }
        else if (bPress) {
            clawMotor.setTargetPosition(CLAW_MAX_POS);
            clawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            clawMotor.setPower(3);
        }
        else {
            clawMotor.setPower(0);
        }

        telemetry.addData("Lift Position:", liftMotor.getCurrentPosition());
        telemetry.addData("Claw Position:", clawMotor.getCurrentPosition());
        telemetry.update();
    }
}
