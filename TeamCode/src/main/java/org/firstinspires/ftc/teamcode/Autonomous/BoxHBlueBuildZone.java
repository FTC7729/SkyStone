package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name = "Blue: Build Zone Park Only", group = "Park")
public class BoxHBlueBuildZone extends G9F9AutonomousHardwareMap {
    public void runOpMode(){
        int state = 0;
        if (state == 0){
            //init robot
            init(hardwareMap);
            waitForStart();
            state = 1;
        }

        if (state == 1){
            //move foreword 1 inch
            telemetry.addData("State","1");
            telemetry.update();
            goForward(.5,1);
            state = 2;
        }

        if (state == 2) {
            telemetry.addData("State","2");
            telemetry.update();
            //strafe right 30 inches
            strafeRight(.3,36);
            state = 3;
        }

        if (state == 3) {
            telemetry.addData("State","3");
            telemetry.update();
            stopMotors();
            //stop!
        }
    }
}
