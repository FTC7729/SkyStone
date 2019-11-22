package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;


@TeleOp(name = "G9ResetMotors", group = "A")
public class G9ResetMotors extends G9F9TeleOpHandler {
    public void handleGamepad1(Gamepad gamepad){
        boolean dpadUp;
        boolean dpadDown;
        boolean aPress;
        boolean bPress;

        dpadUp = gamepad.dpad_up;
        dpadDown = gamepad.dpad_down;
        aPress = gamepad.a;
        bPress = gamepad.b;

        if (dpadUp){
            liftMotor.setPower(0.4);
            //down
        }else if (dpadDown){
            liftMotor.setPower(-0.4);
            //up

        }else{
            liftMotor.setPower(0);
        }


        if (aPress){
            clawMotor.setPower(0.9);
            //opens(?) claw
        }else if (bPress){
            clawMotor.setPower(-0.9);
            //closes(?) claw
        }else {
            clawMotor.setPower(0);
            //stops(!) claw
        }
        telemetry.addData("Lift Position:", liftMotor.getCurrentPosition());
        telemetry.addData("Claw Position:", clawMotor.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void handleGamepad2(Gamepad gamepad) {

    }


}