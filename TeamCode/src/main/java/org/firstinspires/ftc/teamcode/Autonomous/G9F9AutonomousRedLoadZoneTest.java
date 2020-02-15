package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedLoadZoneTest", group = "A")
public class G9F9AutonomousRedLoadZoneTest extends G9F9AutonomousHardwareMap{
    public void runOpMode(){
        int state = 0;
        if (state == 0){
            //init robot
            init(hardwareMap);
            waitForStart();
            state = 1;
        }

        if (state == 1){ //set claw pos max open
            //move forward 72in - 3 squares
            telemetry.addData("State","1");
            telemetry.update();
            setClawPosition(CLAW_MAX_OPEN,0.5);
            //sleep(5000);
            //stopMotors();
            telemetry.addData("Finished state",1);
            telemetry.update();
            //state = 2;
            sleep(1000); //to give the motor a break...
        }

        /*
        if (state == 2) { //claw pos to skystone
            telemetry.addData("State","2");
            telemetry.update();
            //strafe right 30 inches
            setClawPosition(CLAW_CLOSED_ON_SKYSTONE, .5);
            state = 3;
        }
        if (state == 3) { // lift up one inch
            telemetry.addData("State","3");
            telemetry.update();

            setLiftPosition(LIFT_UP_SKYSTONE,.5);
            stopMotors();
            //stop!
        }
    */

    }

}
