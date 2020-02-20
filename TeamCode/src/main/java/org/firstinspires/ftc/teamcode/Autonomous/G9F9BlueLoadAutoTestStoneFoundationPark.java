package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueStone&ParkFoundation", group = "Stone")
public class G9F9BlueLoadAutoTestStoneFoundationPark extends G9F9AutonomousHardwareMap{
    //autonomous for the redloadzone
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
            setClawPosition(CLAW_MAX_OPEN,1);
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
            setClawPosition(CLAW_CLOSED_ON_SKYSTONE,1);
            state = 4;
        }
        if (state == 4){ //lifts stone so it doesn't drag and maybe release onto build platform if we have time to test
            telemetry.addData("State","4");
            telemetry.update();
            setLiftPosition(LIFT_UP_SKYSTONE_FOUNDATION,.4);
            state = 5;
        }
        if (state == 5){ //move backwards to strafe right under alliance bridge with stone
            telemetry.addData("State","5");
            telemetry.update();
            goBackward(.5,10);
            state = 6;
        }
        if (state == 6){ //move left across bridge to foundation
            telemetry.addData("State","6");
            telemetry.update();
            strafeLeft(.7,85);
            state = 7;
        }
        if (state == 7) { //approach foundation to release stone
            telemetry.addData("State", "7");
            telemetry.update();
            goForward(.5,20);
            state = 8;
        }
        if (state == 8){ //open claw to release stone onto foundation
            telemetry.addData("State","8");
            telemetry.update();
            setClawPosition(CLAW_STONE_OPEN,1);
            state = 9;
        }
        if (state == 9){ //move out of foundation area to park under bridge
            telemetry.addData("State","9");
            telemetry.update();
            goBackward(.5,15);
            state = 10;
        }

        if (state == 10){ //park, moving to the right, underneath the alliance bridge
            telemetry.addData("State","9");
            telemetry.update();
            strafeRight(.7,48);
            state = 11;
        }
        if (state == 11){ //to reset the lift encoder values when CompDrive starts
            telemetry.addData("State","10");
            telemetry.update();
            setLiftPosition(LIFT_BOTTOM_MIN,.4);
            state = 12;
        }
        if (state == 12){ //to reset the claw encoder values when CompDrive starts
            telemetry.addData("State","11");
            telemetry.update();
            setClawPosition(CLAW_MIN_CLOSED,1);
            stopMotors();
        }

    }


}
