package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.RobotRunType;

@Disabled
@Autonomous(name = "AdamMM")
public class fieldCentAuto extends AutoBase {

    public void runOpMode(){

        initRobot(RobotRunType.AUTONOMOUS);

        waitForStart();
        //driveVector(80, 0, 0.6, 0.6,0);

    }
}
