package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.RobotRunType;

@Autonomous(name = "test")
public class TestAuto extends AutoBase {

    public void runOpMode(){

        initRobot(RobotRunType.AUTONOMOUS);

        waitForStart();

        //drive forward to blocks
        driveVector(18, 0, 0.5, 0, 0);
        //scan blocks
        //NEED METHOD
        //turn 90 degrees to the right
        turnHeading(0.3, -90);
        turnHeading(0.15, -90);
        //drive
        driveVector(8, -90, 0.5, 0, -90);
        driveVector(20, 0, 0.4, 0, -90);

        lColl.setPower(1);
        rColl.setPower(1);

        turnHeading(0.3, -90);
        turnHeading(0.15, -90);

        driveVector(7, 90, 0.4, 0, -90);
        waitSec(1);
        lColl.setPower(0);
        rColl.setPower(0);

        driveVector(20.5, 180, 0.5, 0, -90);
        turnHeading(0.3, 90);
        turnHeading(0.15, 90);

        driveVector(40, -90, 0.5, 0, 90);

        lColl.setPower(-1);
        rColl.setPower(-1);
        waitSec(2);
        lColl.setPower(0);
        rColl.setPower(0);

        driveVector(40, 90, 0.7, 0, 90);







    }
}
