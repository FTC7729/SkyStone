package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.hardware.Gamepad;

public abstract class G9F9TeleOpHandler extends G9F9TeleOpHardwareMap {
    @Override
    public void init(){ init(hardwareMap); }

    @Override
    public void loop(){
        handleGamepad1(gamepad1);
        handleGamepad2(gamepad2);
    }

    public abstract void handleGamepad1(Gamepad gamepad);

    public abstract void handleGamepad2(Gamepad gamepad);

}
