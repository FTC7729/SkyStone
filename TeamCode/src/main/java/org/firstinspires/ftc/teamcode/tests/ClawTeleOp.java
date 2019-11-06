package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "LiftTestByBandrew", group = "Teleop")
//@Disabled
public class LiftTestTeleOp extends LiftTest{
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void init() {
        init(hardwareMap);
    }

    @Override
    public void loop() {
        final double INCREMENT = 0.01;
        boolean isButtonB= gamepad1.b;
        boolean isButtonA = gamepad1.a;
        boolean isButtonX = gamepad1.x;
        double speed = 0.3;
        float leftStickY = Range.clip(-gamepad1.left_stick_y, -1, 1);

        if (isButtonA) {
            liftMotor.setPower(speed);
            telemetry.addData("Button","A");
            //A is retract
        } else if (isButtonB) {
            liftMotor.setPower(-speed);
            telemetry.addData("Button","B");
            //B is extend
        } else if (isButtonX) {
            liftMotor.setPower(1);
            telemetry.addData("Button","X");
            //X is retract, but with full power
        } else {
            telemetry.addData("Button","None");
            liftMotor.setPower(0);
        }
        telemetry.addData("Lift Position",String.format("%7d", liftMotor.getCurrentPosition()));
        telemetry.update();
    }

















}
