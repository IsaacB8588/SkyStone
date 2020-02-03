package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.RobotRunType;
import org.firstinspires.ftc.teamcode.Tests.SkystonePosition;

@Autonomous(name = "blue - 2stone - 0 - bridge")
public class BlueTwoBlock0Bridge extends AutoBase {

    public void runOpMode() {

        initRobot(RobotRunType.AUTONOMOUS);
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
        }

        waitForStart();

        driveVectorRot(24, 0, 0.5, 0, 0, true);
        waitSec(1);
        SkystonePosition position = getSkyPos(true);
        grab.setPosition(0.6);

        if (position == SkystonePosition.LEFT){

            driveVector(19, 90, 0.5, 0, 0, true);
            turnHeading(0.4, 45);
            setCollect(-0.6);
            driveVector(24, 0, 0.3, 0, 45, false);
            setCollect(0);
            turnHeading(0.4, 90);
            driveVector(20, 180, 0.6, 0, 90, true);
            driveVector(66, -90, 0.5, 0, 90, true);
            setCollect(0.8);
            turnHeading(0.5, 90);
            driveVector(64, 90, 0.5, 0, 90, false);
            setCollect(0);
            turnGlobal(0.4, -45);
            setCollect(-0.6);
            driveVector(24, 0, 0.3, 0, -45, false);
            setCollect(0);
            turnGlobal(0.4, 90);
            driveVector(16, 180, 0.6, 0, 90, true);
            driveVector(64, -90, 0.7, 0, 90, true);
            setCollect(0.8);
            turnHeading(0.5, 90);
            driveVector(16, 90, 0.5, 0, 90, true);
            setCollect(0);

            stop();

        } else if (position == SkystonePosition.CENTER){

            driveVector(24.5, 90, 0.4, 0, 0, true);
            turnHeading(0.4, 45);
            setCollect(-0.6);
            driveVector(24, 0, 0.3, 0, 45, false);
            setCollect(0);
            turnHeading(0.4, 90);
            driveVector(20, 180, 0.4, 0, 90, true);
            driveVector(72, -90, 0.5, 0, 90, true);
            setCollect(0.8);
            driveVector(27, 90, 0.5, 0, 90, true);
            setCollect(0);
            stop();

        } else {

            driveVector(34, 90, 0.4, 0, 0, true);
            turnHeading(0.4, 45);
            setCollect(-0.6);
            driveVector(24, 0, 0.3, 0, 45, false);
            setCollect(0);
            turnHeading(0.4, 90);
            driveVector(20, 180, 0.4, 0, 90, true);
            driveVector(80, -90, 0.5, 0, 90, true);
            setCollect(0.8);
            driveVector(27, 90, 0.5, 0, 90, true);
            setCollect(0);
            stop();

        }

    }

}
