package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.teamcode.teleOp.BoxHAutonomousHardwareMap;

public class BoxHRedBuildZone extends BoxHAutonomousHardwareMap {
    public void runOpMode(){
        int state = 0;
        if (state == 0){
            //init robot

            state = 1;
        }

        if (state == 1){
            //move foreword 1 inch

            state = 2;
        }

        if (state == 2) {
            //strafe left 30 inches

            state = 3;
        }

        if (state == 3) {
            //stop!
        }
    }
}