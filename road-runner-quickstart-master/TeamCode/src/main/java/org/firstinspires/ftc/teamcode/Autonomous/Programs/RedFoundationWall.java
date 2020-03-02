package org.firstinspires.ftc.teamcode.Autonomous.Programs;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.Autonomous.mecanum.RoadRunnerBase;
import org.firstinspires.ftc.teamcode.Autonomous.mecanum.RoadRunnerHardware;
import org.firstinspires.ftc.teamcode.RobotRunType;

@Autonomous(name = "Red-Foundation-0-Wall")
public class RedFoundationWall extends AutoBase {

    @Override
    public void runOpMode() throws InterruptedException {

        initRobot(RobotRunType.AUTONOMOUS);
        RoadRunnerBase drive = new RoadRunnerHardware(hardwareMap);

        drive.setPoseEstimate(new Pose2d(36, -63, Math.toRadians(-90)));

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(48, -36, Math.toRadians(-90)))
                        .build()
        );

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .back(6)
                        .build()
        );

        setHooks(true);
        waitSec(0.5);

        drive.turnSync(Math.toRadians(-15));

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(30)
                        .build()
        );

        drive.turnSync(Math.toRadians(-75));

        setHooks(false);
        waitSec(0.25);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .back(20)
                        .build()
        );

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(4)
                        .strafeLeft(26)
                        .build()
        );

        drive.setPoseEstimate(new Pose2d(42, -63, Math.toRadians(180)));

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeRight(3)
                        .build()
        );

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(6, -60, Math.toRadians(180)))
                        .build()
        );

    }
}
