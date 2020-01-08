package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.RobotRunType;


@Disabled
@Autonomous(name = "Test")
public class fieldCentAuto extends AutoBase {

    public void runOpMode(){

        initRobot(RobotRunType.AUTONOMOUS);

        waitForStart();
        driveVector(18, 0, 0.4, 0, 0, true);
        turnHeading(0.3, -45);
        lColl.setPower(1);
        rColl.setPower(1);
        driveVectorRot(15, 0, 0.4, 0, 45 , false);
        turnHeading(0.3, 0);
        driveVector(15, 180, 0.3, 0, 45, false);
        lColl.setPower(0);
        rColl.setPower(0);


    }
}
