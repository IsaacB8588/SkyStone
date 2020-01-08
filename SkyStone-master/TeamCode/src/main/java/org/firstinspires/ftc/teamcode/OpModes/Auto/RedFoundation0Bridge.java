package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.RobotRunType;

@Autonomous(name = "red - foundation - 0 - bridge")
public class RedFoundation0Bridge extends AutoBase {

    public void runOpMode(){
        initRobot(RobotRunType.AUTONOMOUS);

        waitForStart();

        driveVector(20, 270, 0.3, 0, 0, true);
        driveVector(31.7, 180, 0.2, 0, 0, true);
        setHooks(true);
        waitSec(1);
        driveVector(32, 0, 0.35, 0, 0, false);
        drive(0);
        setHooks(false);
        turnHeading(0.2, 0);
        driveVector(42, 90, 0.5, 0, 0, false);
        driveVector(19, 180, 0.3, 0, 0, false);
        driveVector(17, 90, 0.5, 0, 0 , false);


    }

}
