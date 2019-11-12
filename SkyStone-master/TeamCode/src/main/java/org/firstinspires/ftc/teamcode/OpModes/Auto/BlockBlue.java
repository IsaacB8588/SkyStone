package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.RobotRunType;
import org.firstinspires.ftc.teamcode.Tests.SkystonePosition;

@Autonomous(name = "blue skystone")
public class BlockBlue extends AutoBase {

    public void runOpMode(){

        initRobot(RobotRunType.AUTONOMOUS);

        String position = "";
        waitForStart();

        //drive forward to blocks
        driveVector(22, 0, 0.3, 0, 0);
        turnHeading(0.15, 0);

        while(distanceL.getDistance(DistanceUnit.CM) > 4 && distanceR.getDistance(DistanceUnit.CM) > 4 && opModeIsActive()){

            drive(0.12);

        }
        drive(0);

        waitSec(1);
        //scan blocks
        SkystonePosition scan = skystonePosition();
        //waitSec(0.5);
        driveVector(5, 180, 0.3, 0,0);
        if (scan == SkystonePosition.RIGHT){
            turnHeading(0.3, -90);
            turnHeading(0.15, -90);
            driveVector(21.5, -90, 0.3, 0, -90);
            setCollect(1);
            driveVector(4, 0, 0.2, 0, -90);
            waitSec(0.3);
            setCollect(0);
            driveVector(16, 90, 0.3, 0, -90);
            turnHeading(0.3, 90);
            turnHeading(0.15, 90);
            driveVector(50, 0, 0.5, 0, 90);
            setCollect(-0.5);
            waitSec(0.5);
            setCollect(0);
            driveVector(12, 180, 0.5, 0, 90);


        } else if (scan == SkystonePosition.CENTER){

            turnHeading(0.3, -90);
            turnHeading(0.15, -90);
            driveVector(8, 180, 0.2, 0, -90);
            driveVector(21.5, -90, 0.3, 0, -90);
            setCollect(1);
            driveVector(4, 0, 0.2, 0, -90);
            waitSec(0.3);
            setCollect(0);
            driveVector(16, 90, 0.3, 0, -90);
            turnHeading(0.3, 90);
            turnHeading(0.15, 90);
            driveVector(42, 0, 0.5, 0, 90);
            setCollect(-0.5);
            waitSec(0.5);
            setCollect(0);
            driveVector(10, 180, 0.5, 0, 90);

        } else {

            turnHeading(0.3, 90);
            turnHeading(0.15, 90);
            driveVector(9, 180, 0.2, 0, -90);
            driveVector(21, 90, 0.3, 0, -90);
            setCollect(1);
            driveVector(4, 0, 0.2, 0, -90);
            waitSec(0.3);
            setCollect(0);
            turnHeading(0.15, 90);
            driveVector(17, -90, 0.3, 0, -90);
            turnHeading(0.15, 90);
            driveVector(50, 0, 0.5, 0, 90);
            setCollect(-0.5);
            waitSec(0.5);
            setCollect(0);
            driveVector(12, 180, 0.5, 0, 90);

        }

        stop();
    }
}
