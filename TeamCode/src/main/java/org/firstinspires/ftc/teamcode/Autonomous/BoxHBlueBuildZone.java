package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.teamcode.teleOp.BoxHAutonomousHardwareMap;

public class BoxHBlueBuildZone extends BoxHAutonomousHardwareMap {
    public void runOpMode(){
        int state = 0;
        if (state == 0){
            //init robot
            init(hardwareMap);
            state = 1;
        }

        if (state == 1){
            //move foreword 1 inch
            encoderDrive(.5 , 1,1,1,1,5);
            state = 2;
        }

        if (state == 2) {
            //strafe right 30 inches
            strafeRightEncoder(.5,30,20);
            state = 3;
        }

        if (state == 3) {
            //stop!
        }
    }
}
