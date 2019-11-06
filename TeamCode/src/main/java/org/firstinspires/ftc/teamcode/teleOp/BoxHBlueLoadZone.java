package org.firstinspires.ftc.teamcode.teleOp;

public class BoxHBlueLoadZone extends BoxHAutonomousHardwareMap {
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
