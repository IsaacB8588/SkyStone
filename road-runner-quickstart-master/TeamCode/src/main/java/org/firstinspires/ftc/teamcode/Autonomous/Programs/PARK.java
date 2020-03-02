package org.firstinspires.ftc.teamcode.Autonomous.Programs;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.Autonomous.mecanum.RoadRunnerBase;
import org.firstinspires.ftc.teamcode.Autonomous.mecanum.RoadRunnerHardware;
import org.firstinspires.ftc.teamcode.RobotRunType;
import org.firstinspires.ftc.teamcode.SkystonePosition;

@Autonomous(name = "park")
public class PARK extends AutoBase {

    @Override
    public void runOpMode() throws InterruptedException {

        initRobot(RobotRunType.AUTONOMOUS);
        RoadRunnerBase drive = new RoadRunnerHardware(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        stoneGrab.setPosition(0);
        stoneArm.setPosition(0);

        waitSec(1);

    }
}
