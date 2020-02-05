package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.hardware.Gamepad;

public abstract class G9F9TeleOpHandler_noWheels extends G9F9TeleOpHardwareMap_noWheels {

    @Override
    public void init(){
        init(hardwareMap);
        telemetry.addLine("Initializing! :)");
    }


    @Override
    public void loop(){
        handleGamepad1(gamepad1);
        handleGamepad2(gamepad2);
    }



    public abstract void handleGamepad1(Gamepad gamepad);

    public abstract void handleGamepad2(Gamepad gamepad);

}
