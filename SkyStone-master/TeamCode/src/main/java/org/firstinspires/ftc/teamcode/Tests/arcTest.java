package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.RobotRunType;

@Disabled
@Autonomous(name = "AdamMM")
public class arcTest extends AutoBase {

    public void runOpMode(){

        initRobot(RobotRunType.AUTONOMOUS);

        waitForStart();
        turnArc(18, 0.5, 90);

    }
}
