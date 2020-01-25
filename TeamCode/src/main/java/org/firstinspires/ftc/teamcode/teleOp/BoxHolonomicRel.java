package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
//@Disabled
public class BoxHolonomicRel extends BoxHTeleOpHandler{
    double prevTime = 0;
    double startTime = System.currentTimeMillis();
    double prevLStick = 0;
    double prevMag1 = 0;
    double prevMag2 = 0;
    double timeSinceLastIncrement = 0;
    final double timeIncrement = 50;
    final double powIncrement = 0.1;
    final double deadZone = 0.1;

    public void handleGamepad1(Gamepad gamepad){
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
       float lTrigger;
       float rTrigger;
       boolean dpadLeft;
       boolean dpadRight;
       boolean dpadUp;
       boolean dpadDown;
    
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
        currentAngle = (double) angles.firstAngle;
        //telemetry.addData("Heading: ", "%.3f", currentAngle);
        //telemetry.addData("Loop time (milliseconds): ", "%.3f", loopTime);
        //telemetry.addData("Total time since start (milliseconds): ", "%.3f", timeSinceStart);
        //telemetry.addData("Time since last Increment (milliseconds): ", "%.3f", timeSinceLastIncrement);

        rStickX = -gamepad.right_stick_x;
       rStickY = -gamepad.right_stick_y;
       lStickX = gamepad.left_stick_x;

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


       //lTrigger = gamepad.left_trigger;
       //rTrigger = gamepad.right_trigger;
       dpadLeft = gamepad.dpad_left;
       dpadRight = gamepad.dpad_right;
       dpadUp = gamepad.dpad_up;
       dpadDown = gamepad.dpad_down;

       if (lStickX > 0) {
           lStickX = Math.pow(lStickX, 1.6);
       }else{
           lStickX = -Math.pow(-lStickX, 1.6);
       }
       targetAngle = (Math.atan2(rStickY,rStickX));
       totalAngle = targetAngle + currentAngle;

       if(dpadLeft){
            strafeLeft(0.5);
       } else if(dpadRight){
            strafeRight(0.5);
       } else if(dpadUp){
           goForward(0.5);
       } else if(dpadDown){
            goBackward(0.5);
       } else{
           stopMotors();
       }

       rotationPower = -lStickX;
       mag1 = Math.sqrt(Math.pow(rStickX,2) + Math.pow(rStickY,2)) * (Math.sin(totalAngle + Math.PI / 4));
       mag2 = Math.sqrt(Math.pow(rStickX,2) + Math.pow(rStickY,2)) * (Math.sin(totalAngle - Math.PI / 4));

       maxPower = Math.max(Math.abs(mag1) +  Math.abs(rotationPower) , Math.abs(mag2) +  Math.abs(rotationPower));

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
