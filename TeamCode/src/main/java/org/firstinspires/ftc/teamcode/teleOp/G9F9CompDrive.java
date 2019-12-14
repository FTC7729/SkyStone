package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Holonomic Drive", group="A")
public class G9F9CompDrive extends G9F9TeleOpHandler {
    //lift values are starting from the bottom
    //LIFT_MAX_POS should be positive
    public final int LIFT_MAX_POS = 9000;

    public final int LIFT_MIN_POS = 0;

    public final int LIFT_ON_FOUNDATION = 1121;

    public final int CLAW_MAX_POS = -3500;

    public final int CLAW_MIN_POS = -400;

    public final double ROTATION_CONSTANT = 0.4;

    double subtractAngle = 0;

    double speedScale = 1;
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

        if (dpadUp && liftMotor.getCurrentPosition() < LIFT_MAX_POS){
            liftMotor.setPower(1);
            //down
        }else if (dpadDown && liftMotor.getCurrentPosition() > LIFT_MIN_POS){
            liftMotor.setPower(-1);
            //up

        }else if (xPress){

            //moves lift to foundation
             if (liftMotor.getCurrentPosition() > LIFT_ON_FOUNDATION){
              liftMotor.setPower(-0.5);
             }
             else if (liftMotor.getCurrentPosition() < LIFT_ON_FOUNDATION){
               liftMotor.setPower(0.5);
             }
        }else {
            liftMotor.setPower(0);
        }

        //see above comment
        if (aPress && clawMotor.getCurrentPosition() < CLAW_MIN_POS){
            clawMotor.setPower(1);
        }else if (bPress && clawMotor.getCurrentPosition() > CLAW_MAX_POS){
            clawMotor.setPower(-1);
        }else {
            clawMotor.setPower(0);
            //stops(!) claw
        }
        telemetry.addData("Lift Position:", liftMotor.getCurrentPosition());
        telemetry.addData("Claw Position:", clawMotor.getCurrentPosition());
        telemetry.update();
    }
}
