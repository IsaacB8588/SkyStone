package org.firstinspires.ftc.teamcode.Autonomous.Programs;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.Autonomous.mecanum.RoadRunnerBase;
import org.firstinspires.ftc.teamcode.Autonomous.mecanum.RoadRunnerHardware;
import org.firstinspires.ftc.teamcode.RobotRunType;
import org.firstinspires.ftc.teamcode.SkystonePosition;

@Autonomous(name = "Blue-Skystone-0-Bridge")
public class BlueSkystone extends AutoBase {

    @Override
    public void runOpMode() throws InterruptedException {

        initRobot(RobotRunType.AUTONOMOUS);
        RoadRunnerBase drive = new RoadRunnerHardware(hardwareMap);
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
        }

        drive.setPoseEstimate(new Pose2d(-24, 63, Math.toRadians(-90)));

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(24)
                        .build()
        );


        waitSec(2);
        SkystonePosition pos = getSkyPos(true);
        stoneGrab.setPosition(0);

        double sky1 = 0;
        double sky2 = 0;

        if (pos == SkystonePosition.LEFT){
            sky1 = -52;
            sky2 = -29;
        } else if(pos == SkystonePosition.CENTER) {
            sky1 = -62;
            sky2 = -37;
        } else {
            sky1 = -62;
            sky2 = -45;
        }

        //drive.turnSync(Math.toRadians(-90));

        drive.turnSync(Math.toRadians(-90));

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                    .splineTo(new Pose2d(sky1, 34, Math.toRadians(180)))
                    .build()
        );

        grab();

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(0, 42, Math.toRadians(180)))

                        .splineTo(new Pose2d(14, 42, Math.toRadians(180)))
                        .build()
        );

        drop();
        stoneGrab.setPosition(0.6);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(0, 42, Math.toRadians(180)))
                        .splineTo(new Pose2d(sky2, 35, Math.toRadians(180)))
                        .build()
        );

        grab();

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(0, 42, Math.toRadians(180)))

                        .splineTo(new Pose2d(14, 42, Math.toRadians(180)))
                        .build()
        );

        drop();
        stoneGrab.setPosition(0.6);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(0, 38, Math.toRadians(180)))
                        .build()
        );

    }
}
