package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="BoxyRed1SamplingAndParking",group="tests")
@Disabled
public class BoxHStrafeToLineRed extends BoxHAutonomousHardwareMap {

    public void runOpMode() throws InterruptedException{
        telemetry.addData("Status", "Initialized");
        //initialize the bot
        telemetry.update();

        init(hardwareMap);


        //wait till start here in the this place
        waitForStart();
        // continues to run until you run out of time or hit stop
        while(opModeIsActive()) {

            // STATE 1
            goForward(0.75);
            sleep(3000);
            stopMotors();
            strafeLeft(0.75);
            sleep(3000);
            stopMotors();
            //get reason for idle() later
            idle();
        }
    }
}