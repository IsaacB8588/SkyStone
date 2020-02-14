package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.RobotRunType;
import org.firstinspires.ftc.teamcode.Tests.SkystonePosition;

@Autonomous(name = "blue - stone - 0 - bridge")
public class BlueStone0Bridge extends AutoBase {

    public void runOpMode() {

        initRobot(RobotRunType.AUTONOMOUS);
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
        }

        waitForStart();

        driveVectorRot(24, 0, 0.5, 0, 0, true);
        waitSec(1.5);
        SkystonePosition position = getSkyPos(true);
        stoneArm.setPosition(0.1);
        stoneGrab.setPosition(0);

        double distance1;
        double direction1;
        double distance2;
        double distance3;
        double distance4;

        if (position == SkystonePosition.LEFT){

            distance1 = 1;
            direction1 = -90;
            distance2 = 45;
            distance3 = 20;
            distance4 = 62;

        } else if (position == SkystonePosition.CENTER){

            distance1 = 9;
            direction1 = 90;
            distance2 = 53;
            distance3 = 10;
            distance4 = 72;

        } else {

            distance1 = 19;
            direction1 = 90;
            distance2 = 61;
            distance3 = 0;
            distance4 = 78;

        }

        //hookL.setPosition(0.7);
        driveVector( distance1, direction1, 0.5, 0, 0, true);
        turnGlobal(0.5, 180);
        turnGlobal(0.2, 180);
        driveVector(10, 0, 0.4, 0, 180, true);
        stoneArm.setPosition(0);
        waitSec(0.2);
        stoneGrab.setPosition(0.7);
        waitSec(0.5);
        stoneArm.setPosition(0.5);
        waitSec(0.25);
        driveVector(6, 180, 0.4, 0, 180, true);
        turnGlobal(0.5, 270);
        turnGlobal(0.2, 270);
        driveVector(distance2, -90, 0.5, 0, 270, true);
        stoneArm.setPosition(0);
        waitSec(0.25);
        stoneGrab.setPosition(0);
        waitSec(0.25);
        stoneArm.setPosition(1);

        driveVector(74, 90, 0.5, 0, 265, false);
        turnGlobal(0.5, 180);
        turnGlobal(0.2, 180);
        driveVector(14, 90, 0.3, 0, 180, false);
        driveVector(distance3, -90, 0.5, 0, 180, true);
        stoneArm.setPosition(0);
        waitSec(0.5);

        driveVector(8, 0, 0.4, 0, 180, true);
        waitSec(0.2);
        stoneGrab.setPosition(0.7);
        waitSec(.5);
        stoneArm.setPosition(0.5);
        waitSec(0.5);
        driveVector(8, 180, 0.5, 0, 180, true);
        turnGlobal(0.5, 270);
        turnGlobal(0.2, 270);
        driveVector(distance4, -90, 0.5, 0, 265, true);
        stoneArm.setPosition(0);
        waitSec(0.25);
        stoneGrab.setPosition(0);
        waitSec(0.4);
        driveVector(20, 90, 0.5, 0, 270, true);










        stop();

    }

}
