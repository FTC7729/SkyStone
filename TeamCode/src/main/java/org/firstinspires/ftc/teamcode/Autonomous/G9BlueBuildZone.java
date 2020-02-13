package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class G9BlueBuildZone extends G9F9AutonomousHardwareMap {
    public void runOpMode(){
        int state = 0;
        if (state == 0){
            //init robot
            init(hardwareMap);
            waitForStart();
            state = 1;
        }
        if (state == 1){
            liftMotor.setTargetPosition(2000);
            liftMotor.setPower(0.5);
            state = 2;
        }
        if (state == 2){
            goForward(0.5,6);
            state = 3;
        }
        stopMotors();
        sleep(5000);
        if (state == 3){
            strafeLeft(0.5, 6);
            state = 4;
        }
        stopMotors();
        sleep(5000);
        if (state == 4){
            goForward(0.5, 20);
            state = 5;
        }
        if (state == 5){
            liftMotor.setTargetPosition(700);
        }
    }
}
