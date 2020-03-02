package org.firstinspires.ftc.teamcode.Autonomous.Testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.Autonomous.mecanum.RoadRunnerBase;
import org.firstinspires.ftc.teamcode.Autonomous.mecanum.RoadRunnerHardware;
import org.firstinspires.ftc.teamcode.RobotRunType;

@Config
@Autonomous(group = "tester")
public class GrabTimeTuner extends AutoBase {

    public static double downTime = 0.5;
    public static double grabTIme = 0.5;
    public static double upTime = 0.5;
    public static double releaseTime = 0.5;
    public static double grabValue = 1;

    @Override
    public void runOpMode() throws InterruptedException{

        initRobot(RobotRunType.AUTONOMOUS);
        RoadRunnerBase drive = new RoadRunnerHardware(hardwareMap);
        stoneGrab.setPosition(0);

        waitForStart();


        stoneArm.setPosition(0.1);
        waitSec(0.15);
        stoneGrab.setPosition(grabValue);
        waitSec(0.4);
        stoneArm.setPosition(0.6);
        waitSec(0.4);

        waitSec(2);

        stoneArm.setPosition(0.1);
        waitSec(0.2);
        stoneGrab.setPosition(0);
        waitSec(0.2);
        stoneArm.setPosition(0.6);
        waitSec(0.3);

        waitSec(2);


    }

}
