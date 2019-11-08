package org.firstinspires.ftc.teamcode.HocoBot;

import com.qualcomm.robotcore.hardware.Gamepad;

public abstract class HoCoBotTeleOpHandler extends HoCoBotTeleOpHardwareMap {
    @Override
    public void init(){
        init(hardwareMap);
    }

    @Override
    public void loop(){
        handleGamepad1(gamepad1);
    }

    // BoxH teleop requires only 1 gamepad
    public abstract void handleGamepad1(Gamepad gamepad);
}
