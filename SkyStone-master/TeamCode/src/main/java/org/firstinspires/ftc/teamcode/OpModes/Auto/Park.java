package org.firstinspires.ftc.teamcode.OpModes.Auto;

import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.RobotRunType;

public class Park extends AutoBase {

    public void runOpMode(){
        initRobot(RobotRunType.AUTONOMOUS);

        waitForStart();
        waitSec(20);
        stoneArm.setPosition(0);
        waitSec(0.5);
        stoneGrab.setPosition(0);
        waitSec(1);

        stop();
    }
}
