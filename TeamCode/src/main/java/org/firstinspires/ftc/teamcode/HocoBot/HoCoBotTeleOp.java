package org.firstinspires.ftc.teamcode.HocoBot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
@TeleOp
@Disabled
public class HoCoBotTeleOp extends HoCoBotTeleOpHandler {
    public void handleGamepad1(Gamepad gamepad){
        double rStickX;
        double rStickY;
        double lStickX;
        double lStickY;
        double targetAngle;
        double mag1;
        double mag2;
        double rotationPower;
        double maxPower;
        double scaleDown;

        rStickY = gamepad.right_stick_y;
        lStickY = gamepad.left_stick_y;



        leftMotor.setPower(lStickY);
        rightMotor.setPower(rStickY);

    }
}
