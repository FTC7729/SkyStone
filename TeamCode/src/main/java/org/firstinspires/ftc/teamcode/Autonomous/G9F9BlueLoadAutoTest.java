package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueStone&Park", group = "Stone")
public class G9F9BlueLoadAutoTest extends G9F9AutonomousHardwareMap{
    //autonomous for the BlueLoadZone
    public void runOpMode(){
        int state = 0;
        if (state == 0){
            //init robot
            init(hardwareMap);
            waitForStart();
            //initvuforia(); //this can take a few seconds to init so think about removing this to save time
            state = 1;
        }

        if (state == 1){    //prepares the claw to grab stone
            telemetry.addData("State","1");
            telemetry.update();
            setClawPosition(CLAW_MAX_OPEN,.8);
            // updatevuforia();
            //encoderDrive(.5 , 72,72,72,72,5);
            state = 2;
        }
        if (state == 2) {   //moves forward to grab stone ... watch for speed of robot
            telemetry.addData("State","2");
            telemetry.update();
            goForward(.4,38);
            //strafeLeftEncoder(.5,48,20);      ....maybe implement later if we have time
            state = 3;
        }
        if (state == 3) { //grabs stone to move across the bridge
            telemetry.addData("State","3");
            telemetry.update();
            setClawPosition(CLAW_CLOSED_ON_SKYSTONE,.8);
            state = 4;
        }
        if (state == 4){ //lifts stone so it doesn't drag and maybe release onto build platform
            telemetry.addData("State","4");
            telemetry.update();
            setLiftPosition(LIFT_UP_SKYSTONE,.4);
            state = 5;
        }
        if (state == 5){ //move backwards to strafe right under alliance bridge with stone
            telemetry.addData("State","5");
            telemetry.update();
            goBackward(.5,20);
            state = 6;
        }
        if (state == 6){ //move across bridge to drop off stone
            telemetry.addData("State","6");
            telemetry.update();
            strafeLeft(.5,65);
            state = 7;
        }
        if (state == 7){ //open claw to release stone
            telemetry.addData("State","7");
            telemetry.update();
            setClawPosition(CLAW_MAX_OPEN,.8);
            state = 8;
        }
        if (state == 8){ //turn to the LEFT 90 degrees, facing the back wall
            telemetry.addData("State","8");
            telemetry.update();
            gyroTurn(.5,90);  //target >0 turns to the left while <0 rotates to the right
            state = 9;
        }
        if (state == 9){ //park underneath the bridge - 5 points
            telemetry.addData("State","9");
            telemetry.update();
            goBackward(.5,20);
            state = 10;
        }
        if (state == 10){ //done to reset the lift encoder values when teleop starts
            telemetry.addData("State","10");
            telemetry.update();
            setLiftPosition(LIFT_BOTTOM_MIN,.4);
            stopMotors();
        }
        //think about adding a state to reorient robot forward for teleop vv

        /*if (state == 11){
            telemetry.addData("State","11");
            telemetry.update();
            gyroturn(.5,-90);
            stopMotors();
        } */
    }


}
