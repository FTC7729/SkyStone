package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "G9F9RobotBlueBuildMoveFoundation", group = "Foundation")
public class G9BlueBuildRobertZone extends G9F9AutonomousHardwareMap {
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
            setLiftPosition(2000, 0.5);
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
            strafeLeft(0.4, 14);
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
            goForward(0.4, 26);
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
            strafeRight(0.5,16);
            state = 8;
        }
        if (state == 8){
            telemetry.addData("State: ", state);
            telemetry.update();
            gyroTurn(0.5, 100);
            gyroTurn(0.5, 90);
            state = 9;
        }
        if (state == 9){
            telemetry.addData("State: ", state);
            telemetry.update();
            strafeLeft(0.5,36);
            state = 10;
        }
        if (state == 10){
            telemetry.addData("State: ", state);
            telemetry.update();
            gyroTurn(0.5, 90);
            state = 11;
        }
        if(state == 11){
            telemetry.addData("State: ", state);
            telemetry.update();
            goForward(0.5, 6);
            state = 12;
        }
        if (state == 12){
            telemetry.addData("State: ", state);
            telemetry.update();
            setLiftPosition(2000, 0.5);
            state = 13;
        }
        if (state == 13){
            telemetry.addData("State: ", state);
            telemetry.update();
            goBackward(0.5, 32);
            state = 14;
        }
        if (state == 14){
            telemetry.addData("State: ", state);
            telemetry.update();
            setLiftPosition(0, 0.5);
            setClawPosition(0, 0.5);
        }


    }
}
