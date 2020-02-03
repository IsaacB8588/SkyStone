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
        grab.setPosition(0.55);

        if (position == SkystonePosition.LEFT){

            driveVector(17, 90, 0.4, 0, 0, true);
            turnHeading(0.4, 45);
            setCollect(-0.6);
            driveVector(24, 0, 0.3, 0, 45, false);
            setCollect(0);
            turnHeading(0.4, 90);
            driveVector(20, 180, 0.4, 0, 90, true);
            driveVector(89, -90, 0.5, 0, 90, true);
            turnGlobal(0.5, 180);
            setCollect(0.8);
            driveVector(12, 0, 0.3, 0, 180, false);
            setHooks(true);
            setCollect(0);
            waitSec(1);
            driveVector(20, 180, 0.5, 0, 180, false);
            turnGlobal(0.5, 270);
            driveVector(8, 270, 0.5, 0, 270, false);
            setHooks(false);
            waitSec(0.5);
            driveVector(55, 70, 0.5, 0, 270, false);
            stop();

        } else if (position == SkystonePosition.CENTER){

            driveVector(26.5, 90, 0.4, 0, 0, true);
            turnHeading(0.4, 45);
            setCollect(-0.6);
            driveVector(24, 0, 0.3, 0, 45, false);
            setCollect(0);
            turnHeading(0.4, 90);
            driveVector(20, 180, 0.4, 0, 90, true);
            driveVector(97, -90, 0.5, 0, 90, true);
            turnGlobal(0.5, 180);
            setCollect(0.8);
            driveVector(12, 0, 0.3, 0, 180, false);
            setHooks(true);
            setCollect(0);
            waitSec(1);
            driveVector(20, 180, 0.5, 0, 180, false);
            turnGlobal(0.5, 270);
            driveVector(8, 270, 0.5, 0, 270, false);
            setHooks(false);
            waitSec(0.5);
            driveVector(55, 70, 0.5, 0, 270, false);

            stop();

        } else {

            driveVector(36, 90, 0.4, 0, 0, true);
            turnHeading(0.4, 45);
            setCollect(-0.6);
            driveVector(24, 0, 0.3, 0, 45, false);
            setCollect(0);
            turnHeading(0.4, 90);
            driveVector(20, 180, 0.6, 0, 90, true);
            driveVector(105, -90, 0.5, 0, 90, true);
            turnGlobal(0.5, 180);
            setCollect(0.8);
            driveVector(12, 0, 0.3, 0, 180, false);
            setHooks(true);
            setCollect(0);
            waitSec(1);
            driveVector(20, 180, 0.5, 0, 180, false);
            turnGlobal(0.5, 270);
            driveVector(8, 270, 0.5, 0, 270, false);
            setHooks(false);
            waitSec(0.5);
            driveVector(55, 70, 0.5, 0, 270, false);
            
            stop();

        }

    }

}
