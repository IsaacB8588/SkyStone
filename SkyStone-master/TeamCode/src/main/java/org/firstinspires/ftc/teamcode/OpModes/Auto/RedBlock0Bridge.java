package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.RobotRunType;
import org.firstinspires.ftc.teamcode.Tests.SkystonePosition;

@Autonomous(name = "red - stone - 0 - bridge")
public class RedBlock0Bridge extends AutoBase {


    public void runOpMode() {

        //initialize hardware and vision tracking
        initRobot(RobotRunType.AUTONOMOUS);
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
        }

        waitForStart();

        //drive up to quarry
        driveVectorRot(24, 0, 0.5, 0, 0, true);
        waitSec(2);
        //scan position of skystone
        SkystonePosition position = getSkyPos(false);
        //lower grabbing mechanism to stop stone
        grab.setPosition(0.55);

        //code for left skystone
        if (position == SkystonePosition.LEFT){

            driveVector(23.5, -90, 0.4, 0, 0, true);
            turnHeading(0.4, -45);
            setCollect(-0.6);
            driveVector(33, 0, 0.3, 0, -45, false);
            setCollect(0);
            turnHeading(0.4, -90);
            driveVector(16, 180, 0.4, 0, -90, true);
            driveVector(76, 90, 0.5, 0, -90, true);
            setCollect(0.8);
            driveVector(27, -90, 0.5, 0, -90, true);
            setCollect(0);
            stop();

            //code for center skystone
        } else if (position == SkystonePosition.CENTER){

            driveVector(13, -90, 0.4, 0, 0, true);
            turnHeading(0.4, -45);
            setCollect(-0.6);
            driveVector(33, 0, 0.3, 0, -45, false);
            setCollect(0);
            turnHeading(0.4, -90);
            driveVector(16, 180, 0.4, 0, -90, true);
            driveVector(68, 90, 0.5, 0, -90, true);
            setCollect(0.8);
            driveVector(27, -90, 0.5, 0, -90, true);
            setCollect(0);
            stop();

            //code for right skystone
        } else {

            driveVector(2, -90, 0.4, 0, 0, true);
            turnHeading(0.4, -45);
            setCollect(-0.6);
            driveVector(33, 0, 0.3, 0, -45, false);
            setCollect(0);
            turnHeading(0.4, -90);
            driveVector(16, 180, 0.4, 0, -90, true);
            driveVector(60, 90, 0.5, 0, -90, true);
            setCollect(0.8);
            driveVector(27, -90, 0.5, 0, -90, true);
            setCollect(0);
            stop();
        }

    }

}
