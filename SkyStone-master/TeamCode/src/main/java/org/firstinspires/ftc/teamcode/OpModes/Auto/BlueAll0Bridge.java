package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.RobotRunType;
import org.firstinspires.ftc.teamcode.Tests.SkystonePosition;

@Autonomous(name = "blue - all - 0 - bridge")
public class BlueAll0Bridge extends AutoBase {

    public void runOpMode() {

        initRobot(RobotRunType.AUTONOMOUS);
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
        }

        waitForStart();

        driveVectorRot(24, 0, 0.5, 0, 0, true);
        waitSec(2);
        SkystonePosition position = getSkyPos(true);
        stoneArm.setPosition(0);
        stoneGrab.setPosition(0);

        double distance1;
        double direction1;
        double distance2;
        double distance3;
        double distance4;

        if (position == SkystonePosition.LEFT){

            distance1 = 1;
            direction1 = -90;
            distance2 = 80;
            distance3 = 20;
            distance4 = 66;

        } else if (position == SkystonePosition.CENTER){

            distance1 = 9;
            direction1 = 90;
            distance2 = 88;
            distance3 = 10;
            distance4 = 76;

        } else {

            distance1 = 19;
            direction1 = 90;
            distance2 = 96;
            distance3 = 0;
            distance4 = 82;

        }

        hookL.setPosition(0.7);
        driveVector( distance1, direction1, 0.5, 0, 0, true);
        turnGlobal(0.5, 180);
        turnGlobal(0.2, 180);
        driveVector(10, 0, 0.4, 0, 180, true);
        waitSec(0.2);
        stoneGrab.setPosition(0.7);
        waitSec(1);
        stoneArm.setPosition(0.5);
        waitSec(0.5);
        driveVector(6, 180, 0.4, 0, 180, true);
        turnGlobal(0.5, 270);
        turnGlobal(0.2, 270);
        driveVector(distance2, -90, 0.5, 0, 270, true);

        turnGlobal(0.5, 180);
        turnGlobal(0.2, 180);
        driveVector(10, 0, 0.4, 0, 180, true);
        stoneArm.setPosition(0);
        waitSec(0.5);
        stoneGrab.setPosition(0);
        waitSec(0.5);
        stoneArm.setPosition(0.5);
        driveVector(9, 90, 0.5, 0, 180, true);
        driveVector(4, 0, 0.4, 0, 180, false);
        setHooks(true);
        waitSec(1);
        driveVector(20, 180, 0.5, 0, 180, false);
        turnGlobal(0.5, 270);
        driveVector(8, 270, 0.5, 0, 270, false);
        setHooks(false);
        driveVector(50, 75, 0.5, 0, 270, false);
        stoneArm.setPosition(0);
        waitSec(0.5);

        stop();

    }

}
