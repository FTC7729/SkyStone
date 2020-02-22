package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Autonomous.BoxHAutonomousHardwareMap;
import org.firstinspires.ftc.teamcode.Autonomous.G9F9AutonomousHardwareMap;

@Autonomous
public class GyroTurnTest extends BoxHAutonomousHardwareMap {
    public void runOpMode(){
        int state = 0;
        if (state == 0){
            //init robot
            init(hardwareMap);
            waitForStart();
            state = 1;
        }

        if (state == 1){
            //telemetry.addData("Turning: ", "-90 degrees");
            //telemetry.update();
            //gyroTurn(0.5,190);
            //strafeLeft(0.5);
            gyroTurn(0.5,90);
        }
        if (state == 2){
/*
            telemetry.addData("Turning: ", "90 degrees");
            telemetry.update();
            gyroTurn(0.5, 0);

*/
            state = 3;
        }
        if (state == 3){
            stopMotors();
        }
        sleep(500);
    }
}
