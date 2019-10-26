package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.hardware.Gamepad;

public abstract class TestingBoxTeleOpHandler extends BoxHAutonomousHardwareMap {
    /*@Override
    public void init(){
        init(hardwareMap);
    }*/

    /*@Override
    public void loop(){
        handleGamepad1(gamepad1);
    }*/

    public void runOpMode() {
        init(hardwareMap);
        waitForStart();
        while(opModeIsActive()){
            handleGamepad1(gamepad1);
        }
    }

    public abstract void handleGamepad1(Gamepad gamepad);
}
