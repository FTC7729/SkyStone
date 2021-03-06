package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.mmPerInch;

@Autonomous(name = "RedStoneWithVuforia&ParkTest", group = "Stone")
public class G9F9RedLoadAutoTest extends G9F9AutonomousHardwareMap{
    //autonomous for the redloadzone
    //144 in is a full field
    //72 half a field
    //36 is a quarter of a field
    public void runOpMode(){
        int state = 0;
        if (state == 0){
            //init robot
            telemetry.setAutoClear(false);
            init(hardwareMap);
            waitForStart();
            //initvuforia(); //this can take a few seconds to init so think about removing this to save time
            state = 1;
        }

        if (state == 1){    //prepares the claw to grab stone
            telemetry.addData("State","1");
            telemetry.update();
            //setClawPosition(CLAW_MAX_OPEN,.8);
            //updatevuforia();
            //encoderDrive(.5 , 72,72,72,72,5);
            state = 2;
        }
        if (state == 2) {   //moves forward to grab stone ... watch for speed of robot
            telemetry.addData("State","2");
            telemetry.update();
            /*if (targetVisible){
                updatevuforia();
                if (translation.get(1)/mmPerInch = )
               */
            goForward(.5,17);
            initvuforia();
            updatevuforia();
            stopMotors();
            //strafeLeftEncoder(.5,48,20);      ....maybe implement later if we have time
            //state = 3;
        }
        /*if (state == 3) { //grabs stone to move across the bridge
            telemetry.addData("State","3");
            telemetry.update();
            setClawPosition(CLAW_CLOSED_ON_SKYSTONE,.8);
            state = 4;
        }
        if (state == 4){ //lifts stone so it doesn't drag and maybe release onto build platform if we have time to test
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
            strafeRight(.5,65);
            state = 7;
        }
        if (state == 7){ //open claw to release stone
            telemetry.addData("State","7");
            telemetry.update();
            setClawPosition(CLAW_MAX_OPEN,.8);
            state = 8;
        }
        if (state == 8){ //turn to the right 90 degrees
            telemetry.addData("State","8");
            telemetry.update();
            gyroTurn(.5,-90);  //turns right, facing the back wall so the stone doesn't get dragged across the line
            state = 9;
        }
        if (state == 9){ //park underneath the alliance bridge = 5 points
            telemetry.addData("State","9");
            telemetry.update();
            goBackward(.5,20);
            state = 10;
        }
        if (state == 10){ //to reset the lift encoder values when CompDrive starts
            telemetry.addData("State","10");
            telemetry.update();
            setLiftPosition(LIFT_BOTTOM_MIN,.4);
            state = 11;
        }
        if (state == 11){ //to reset the claw encoder values when CompDrive starts

            telemetry.addData("State","7");
            telemetry.update();
            setClawPosition(CLAW_MIN_CLOSED,.8);
            stopMotors();
        }
        //think about adding a state to reorient robot forward for teleop vv

        /*if (state == 11){
            telemetry.addData("State","11");
            telemetry.update();
            gyroturn(.5,90);
            stopMotors();
        } */

    }


}
