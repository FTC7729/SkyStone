package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class G9F9AutoTest extends G9F9AutonomousHardwareMap{
    public void runOpMode(){
        int state = 0;
        if (state == 0){
            //init robot
            init(hardwareMap);
            waitForStart();
            initvuforia();
            state = 1;
        }

        if (state == 1){
            //
            telemetry.addData("State","1");
            telemetry.update();
            setClawPosition(CLAW_MAX_OPEN,.8);
            // updatevuforia();
            //encoderDrive(.5 , 72,72,72,72,5);
            state = 2;
        }
        if (state == 2) {
            telemetry.addData("State","2");
            telemetry.update();
            goForward(.4,38);
            //strafe right 30 inches
            //strafeLeftEncoder(.5,48,20);
            state = 3;
        }
        if (state == 3) {
            telemetry.addData("State","3");
            telemetry.update();
            setClawPosition(CLAW_CLOSED_ON_SKYSTONE,.8);
            state = 4;
        }
        if (state == 4){ //lift skystone
            telemetry.addData("State","4");
            telemetry.update();
            setLiftPosition(LIFT_UP_SKYSTONE,.4);
            state = 5;
        }
        if (state == 5){ //move backwards
            telemetry.addData("State","5");
            telemetry.update();
            goBackward(.5,20);
            state = 6;
        }
        if (state == 6){ //move backwards
            telemetry.addData("State","6");
            telemetry.update();
            strafeRight(.5,65);
            state = 7;
        }
        if (state == 7){ //
            telemetry.addData("State","7");
            telemetry.update();
            setClawPosition(CLAW_MAX_OPEN,.8);
            stopMotors();
        }
        /*if (state == 8){ //
            telemetry.addData("State","7");
            telemetry.update();
            setClawPosition(CLAW_MAX_OPEN,.8);
            stopMotors();
        }
        if (state == 9){ //
            telemetry.addData("State","8");
            telemetry.update();
            strafeRight(.5,54);
            stopMotors();
        } */
    }


}
