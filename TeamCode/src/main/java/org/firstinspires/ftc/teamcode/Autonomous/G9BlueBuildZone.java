package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class G9BlueBuildZone extends G9F9AutonomousHardwareMap {
    public void runOpMode(){
        int state = 0;
        if (state == 0){
            //init robot
            telemetry.addData("State: ", state);
            telemetry.update();
            init(hardwareMap);
            waitForStart();

            state = 1;
        }
        if (state == 1){
            telemetry.addData("State: ", state);
            telemetry.update();
            setLiftPosition(1000, 0.5);
            //this claw position maximizes our odds of not hitting the claw against one of the nubs
            setClawPosition(780, 0.5);
            state = 2;
        }
        if (state == 2){
            telemetry.addData("State: ", state);
            telemetry.update();
            goForward(0.4,6);
            state = 3;
        }
        sleep(500);
        if (state == 3){
            telemetry.addData("State: ", state);
            telemetry.update();
            strafeLeft(0.4, 12);
            state = 4;
        }
        sleep(500);
        if (state == 4){
            telemetry.addData("State: ", state);
            telemetry.update();
            gyroTurn(0.5, 0);
            state = 5;
        }
        if (state == 5){
            telemetry.addData("State: ", state);
            telemetry.update();
            goForward(0.4, 24);
            gyroTurn(0.5, 0);
            state = 6;
        }
        sleep(500);
        if (state == 6){
            telemetry.addData("State: ", state);
            telemetry.update();
            setLiftPosition(0, 0.2);
            state = 7;
        }
        if (state == 7){
            telemetry.addData("State: ", state);
            telemetry.update();
            gyroTurn(0.5, 90);
        }
    }
}
