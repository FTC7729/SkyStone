package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class BoxHolonomic extends OpMode{
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    public void init(){
        leftFront  = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftRear");
        rightBack  = hardwareMap.dcMotor.get("rightRear");

        //
        // Reverse the motors on the right side
        //
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
    }
    public void loop(){
        handleGamepad1(gamepad1);

    }
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

       rStickX = gamepad.right_stick_x;
       rStickY = -gamepad.right_stick_y;
       lStickX = gamepad.left_stick_x;

       targetAngle = (Math.atan2(rStickY,rStickX));

       rotationPower = -lStickX;
       mag1 = Math.sqrt(Math.pow(rStickX,2) + Math.pow(rStickY,2)) * (Math.sin(targetAngle + Math.PI / 4));
       mag2 = Math.sqrt(Math.pow(rStickX,2) + Math.pow(rStickY,2)) * (Math.sin(targetAngle - Math.PI / 4));

       maxPower = Math.max(Math.abs(mag1) +  Math.abs(rotationPower) , Math.abs(mag2) +  Math.abs(rotationPower)) + 0.15;
       scaleDown = 1.0;

       if (maxPower > 1)
           scaleDown = 1.0 / maxPower;

        leftFront.setPower((mag2 - rotationPower) * scaleDown);
        leftBack.setPower((mag1 - rotationPower) * scaleDown);
        rightBack.setPower((mag2 + rotationPower) * scaleDown);
        rightFront.setPower((mag1 + rotationPower) * scaleDown);

    }

}
