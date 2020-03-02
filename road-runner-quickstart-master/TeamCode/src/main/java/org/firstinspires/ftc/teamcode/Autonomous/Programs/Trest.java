package org.firstinspires.ftc.teamcode.Autonomous.Programs;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.Autonomous.mecanum.RoadRunnerBase;
import org.firstinspires.ftc.teamcode.Autonomous.mecanum.RoadRunnerHardware;
import org.firstinspires.ftc.teamcode.RobotRunType;
import org.firstinspires.ftc.teamcode.SkystonePosition;

@Autonomous(name = "test-SkystoneOn-0-Bridge")
public class Trest extends AutoBase {

    @Override
    public void runOpMode() throws InterruptedException {

        initRobot(RobotRunType.AUTONOMOUS);
        RoadRunnerBase drive = new RoadRunnerHardware(hardwareMap);

        //initVuforia();
        //initTfod();
        //tfod.activate();


        drive.setPoseEstimate(new Pose2d(-24, 63, Math.toRadians(-90)));

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(24)
                        .build()
        );

        SkystonePosition pos = getSkyPos(true);



    }
}
