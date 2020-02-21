package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "G9F9BlueBuildMoveFoundation", group = "Foundation")
public class G9BlueBuildZone extends G9F9AutonomousHardwareMap {
    public void runOpMode(){
        int state = 0;
        if (state == 0){
            //init robot
            telemetry.addData("State: ", "0");
            telemetry.update();
            init(hardwareMap);
            waitForStart();
            state = 1;
        }
        if (state == 1){ //strafe left to wall
            telemetry.addData("State: ", "1");
            telemetry.update();
            strafeLeft(0.5, 14);
            state = 2;
        }
        if (state == 2){
            telemetry.addData("State: ", "2");
            telemetry.update();
            setLiftPosition(2000, 0.5);
            //this claw position maximizes our odds of not hitting the claw against one of the nubs
            setClawPosition(780, 0.5);
            state = 3;
        }
        if (state == 3){ //move towards foundation
            telemetry.addData("State: ", "3");
            telemetry.update();
            goForward(0.4,38);
            state = 4;
        }
        if (state == 4){
            telemetry.addData("State: ", "4");
            telemetry.update();
            setLiftPosition(LIFT_UP_SKYSTONE_FOUNDATION,.7);
            state = 5;
        }
        if (state == 5) { //strafe left to wall
            telemetry.addData("State: ", "5");
            telemetry.update();
            strafeLeft(0.5, 7);
            state = 6;
        }
        if (state == 6){
            telemetry.addData("State: ", "6");
            telemetry.update();
            setLiftPosition(2000,.7);
            state = 7;
            }
        if (state == 7) { //strafe left to wall
            telemetry.addData("State: ", "7");
            telemetry.update();
            strafeRight(0.5, 62);
            state = 8;
        }
            if (state == 8){ //to reset the claw encoder values when CompDrive starts
                telemetry.addData("State","8");
                telemetry.update();
                setClawPosition(CLAW_MIN_CLOSED,1);
                state = 9;
            }
            if (state == 9){ //to reset the claw encoder values when CompDrive starts
                telemetry.addData("State","9");
                telemetry.update();
                setLiftPosition(LIFT_BOTTOM_MIN,.7);
                stopMotors(); 
            }
    }
}
