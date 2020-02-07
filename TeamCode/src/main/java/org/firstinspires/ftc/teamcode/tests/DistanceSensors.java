package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomous.G9F9AutonomousHardwareMap;
@Autonomous
public class DistanceSensors extends G9F9AutonomousHardwareMap {
    public void runOpMode (){
        while (true){
            //doesnt work (yet!)
            telemetry.addData("right range", String.format("%.01f in", rightDistanceSensor.getDistance(DistanceUnit.INCH)));
            telemetry.addData("left range", String.format("%.01f in", leftDistanceSensor.getDistance(DistanceUnit.INCH)));
            telemetry.update();
        }
    }
}
