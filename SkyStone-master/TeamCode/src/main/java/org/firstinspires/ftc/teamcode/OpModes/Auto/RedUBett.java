package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.RobotRunType;

@Autonomous(name = "red - Ubett - 0 - wall")
public class RedUBett extends AutoBase {

    public void runOpMode(){
        initRobot(RobotRunType.AUTONOMOUS);

        waitForStart();


        driveVector(10, 270, 0.3, 0, 0, true);
        waitSec(15);
        driveVector(31.7, 180, 0.2, 0, 0, true);
        setHooks(true);
        waitSec(1);
        driveVector(37, 0, 0.35, 0, 0, false);
        drive(0);
        setHooks(false);
        turnHeading(0.2, 0);
        driveVector(54, 90, 0.5, 0, 0, false);


    }

}
