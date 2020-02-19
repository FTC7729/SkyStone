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
            //move forward 72in - 3 squares
            telemetry.addData("State","1");
            telemetry.update();
            //encoderDrive(.5 , 72,72,72,72,5);
            state = 2;
        }
        if (state == 2) {
            telemetry.addData("State","2");
            telemetry.update();
            //strafe right 30 inches
            //strafeLeftEncoder(.5,48,20);
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
