package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.RobotRunType;

@Autonomous(name = "logan likes bbc")
public class DecelerationTest extends AutoBase {

    public void runOpMode(){

        initRobot(RobotRunType.AUTONOMOUS);
        waitForStart();

        driveVector(40, 0, 0.6, 0.2, 0, true);
        //driveVector(70, 0, 0.6, 0.3, 0, true);
        turnHeading(0.3, 90);
        driveVector(30, 225, 0.6, 0.2, 90, true);
        turnHeading(0.3, 0);
        driveVector(50, 0, 0.4, 0.4, -90, true);



    }

}
