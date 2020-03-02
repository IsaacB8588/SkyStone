package org.firstinspires.ftc.teamcode.Autonomous.PidTuners;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.mecanum.RoadRunnerBase;
import org.firstinspires.ftc.teamcode.Autonomous.mecanum.RoadRunnerHardware;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
@Disabled
public class StraightTest extends LinearOpMode {
    public static double DISTANCE = 50;

    @Override
    public void runOpMode() throws InterruptedException {
        RoadRunnerBase drive = new RoadRunnerHardware(hardwareMap);

        Trajectory trajectory = drive.trajectoryBuilder()
                .forward(DISTANCE)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(trajectory);
    }
}
