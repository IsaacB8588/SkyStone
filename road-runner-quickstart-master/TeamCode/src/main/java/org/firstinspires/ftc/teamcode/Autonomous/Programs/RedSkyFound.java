package org.firstinspires.ftc.teamcode.Autonomous.Programs;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.Autonomous.mecanum.RoadRunnerBase;
import org.firstinspires.ftc.teamcode.Autonomous.mecanum.RoadRunnerHardware;
import org.firstinspires.ftc.teamcode.RobotRunType;
import org.firstinspires.ftc.teamcode.SkystonePosition;

@Autonomous(name = "Red-SkystoneOn-0-Bridge")
public class RedSkyFound extends AutoBase {

    @Override
    public void runOpMode() throws InterruptedException {

        initRobot(RobotRunType.AUTONOMOUS);
        RoadRunnerBase drive = new RoadRunnerHardware(hardwareMap);
        initVuforia();
        initTfod();

        if (tfod != null){
            tfod.activate();
        }

        drive.setPoseEstimate(new Pose2d(-37, -63, Math.toRadians(90)));

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(24)
                        .build()
        );


        waitSec(2);
        SkystonePosition pos = getSkyPos(false);
        stoneGrab.setPosition(0);

        double sky1 = 0;
        double sky2 = 0;

        if (pos == SkystonePosition.LEFT){
            sky1 = -62;
            sky2 = -38;
        } else if(pos == SkystonePosition.CENTER) {
            sky1 = -54;
            sky2 = -30;
        } else {
            sky1 = -46;
            sky2 = -22;
        }

        drive.turnSync(Math.toRadians(-90));

        drive.followTrajectorySync(
                drive.trajectoryBuilder()

                        .strafeLeft(3.5)
                        .build()
        );

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(sky1, -36, Math.toRadians(0)))
                        .build()
        );

        grab();

        drive.followTrajectorySync(
                drive.trajectoryBuilder()

                        .strafeRight(3.5)
                        .build()
        );

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(0, -42, Math.toRadians(0)))
                        .splineTo(new Pose2d(52, -34, Math.toRadians(0)))
                        .build()
        );

        drop();
        stoneGrab.setPosition(0.6);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(0, -40, Math.toRadians(0)))
                        .splineTo(new Pose2d(sky2, -36, Math.toRadians(0)))
                        .build()
        );

        grab();

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(0, -42, Math.toRadians(0)))
                        .splineTo(new Pose2d(44, -34, Math.toRadians(0)))
                        .build()
        );

        drop();
        stoneGrab.setPosition(0.6);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(0, -38, Math.toRadians(0)))
                        .build()
        );

    }
}
