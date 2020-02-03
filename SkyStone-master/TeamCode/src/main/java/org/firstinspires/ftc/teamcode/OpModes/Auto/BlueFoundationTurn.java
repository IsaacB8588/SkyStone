package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.RobotRunType;

@Autonomous(name = "blue - foundationTurn - 0 - wall")
public class BlueFoundationTurn extends AutoBase {

    public void runOpMode(){
        initRobot(RobotRunType.AUTONOMOUS);

        waitForStart();

        driveVector(16, 90, 0.3, 0, 0, true);
        driveVector(31.7, 180, 0.2, 0, 0, true);
        setHooks(true);
        waitSec(1);
        driveVector(20, 0, 0.35, 0, 0, false);
        turnFoundation(0.45, 90);
        driveVector(8, 90, 0.3, 0, 90, false);
        setHooks(false);
        driveVector(10, 0, 0.4, 0, 90, true);
        turnHeading(0.4, 90);
        driveVector(45, 270, 0.4, 0, 90, true);



    }

}
