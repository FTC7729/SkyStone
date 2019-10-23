package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class BoxHolonomicRel extends BoxHTeleOpHandler{

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

        //
        // Get the current relative heading of the robot
        //
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        currentAngle = (double) angles.firstAngle;
        telemetry.addData("Heading: ", "%.3f", currentAngle);

       rStickX = -gamepad.right_stick_x;
       rStickY = -gamepad.right_stick_y;
       lStickX = gamepad.left_stick_x;
       if (lStickX > 0) {
           lStickX = Math.pow(lStickX, 1.6);
       }else{
           lStickX = -Math.pow(-lStickX, 1.6);
       }
       targetAngle = (Math.atan2(rStickY,rStickX));
       totalAngle = targetAngle + currentAngle;


       rotationPower = -lStickX;
       mag1 = Math.sqrt(Math.pow(rStickX,2) + Math.pow(rStickY,2)) * (Math.sin(totalAngle + Math.PI / 4));
       mag2 = Math.sqrt(Math.pow(rStickX,2) + Math.pow(rStickY,2)) * (Math.sin(totalAngle - Math.PI / 4));

       maxPower = Math.max(Math.abs(mag1) +  Math.abs(rotationPower) , Math.abs(mag2) +  Math.abs(rotationPower));


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
