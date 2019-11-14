package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.BoxHAutonomousHardwareMap;
@Autonomous
public class BoxHRedLoadZone extends BoxHAutonomousHardwareMap {
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
            encoderDrive(.5 , 1,1,1,1,5);
            state = 2;
        }

        if (state == 2) {
            telemetry.addData("State","2");
            telemetry.update();
            //strafe right 30 inches
            strafeRightEncoder(.5,30,20);
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
