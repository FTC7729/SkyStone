package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
@TeleOp
public class BoxCStrafeTest extends TestingBoxTeleOpHandler {
    public void handleGamepad1(Gamepad gamepad){
        telemetry.addData("It started!",":]");
        boolean dpadLeft = gamepad.dpad_left;
        boolean dpadRight = gamepad.dpad_right;
        boolean dpadDown = gamepad.dpad_down;
        if(dpadLeft){
            strafeLeft(0.25);
            telemetry.addData("Strafing","Left");
        }
        if(dpadRight){
            strafeRight(0.25);
            telemetry.addData("Strafing","Right");
        }
        if(dpadDown){
            stopMotors();
        }
        telemetry.update();
    }

}
