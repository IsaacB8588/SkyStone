package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.RobotRunType;

@Autonomous(name = "blue foundation")
public class FoundationBlue extends AutoBase {

    public void runOpMode(){
        initRobot(RobotRunType.AUTONOMOUS);

        waitForStart();

        driveVector(20, 90, 0.3, 0, 180);
        driveVector(31.4, 180, 0.2, 0, 180);
        setHooks(true);
        waitSec(1);
        driveVector(32, 0, 0.25, 0, 0);
        drive(0);
        setHooks(false);
        turnHeading(0.15, 0);
        driveVector(40, -90, 0.4, 0, 180);
        driveVector(18, 180, 0.3, 0, 180);
        driveVector(15, 90, 0.4, 0, 180);
        driveVector(30, -90, 0.4, 0, 180);

    }

}
