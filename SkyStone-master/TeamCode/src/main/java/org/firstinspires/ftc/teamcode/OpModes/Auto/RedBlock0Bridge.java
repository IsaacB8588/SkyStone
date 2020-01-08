package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.RobotRunType;
import org.firstinspires.ftc.teamcode.Tests.SkystonePosition;

@Autonomous(name = "red - stone - 0 - bridge")
public class RedBlock0Bridge extends AutoBase {


    public void runOpMode() {

        initRobot(RobotRunType.AUTONOMOUS);

        String position = "";
        waitForStart();

        //drive forward to blocks
        driveVector(24.2, 0, 0.4, 0, 0, true);
        turnHeading(0.2, 0);

        while (distanceL.getDistance(DistanceUnit.CM) > 4 && distanceR.getDistance(DistanceUnit.CM) > 4 && opModeIsActive()) {
            drive(0.2);
        }
        drive(0);

        waitSec(0.25);
        //scan blocks
        SkystonePosition scan = skystonePosition();
        driveVector(2, 180, 0.2, 0, 0, true);


        if (scan == SkystonePosition.LEFT){

            telemetry.addData("Position: ", "C");
            telemetry.update();

            driveVector(4.75, 90, 0.6, 0, 0, true);
            turnHeading(0.3, 35);
            lColl.setPower(1);
            rColl.setPower(1);
            driveVectorRot(30, 0, 0.4, 0, -35 , false);
            turnHeading(0.3, 0);
            lColl.setPower(0);
            rColl.setPower(0);
            driveVector(17, 180, 0.3, 0, 0, false);


            turnHeading(0.3, -90);
            driveVector(45, 90, 0.7, 0, -90, true);
            setCollect(-0.6);

            waitSec(2);
            setCollect(0);
            lColl.setPower(0);
            rColl.setPower(0);

            driveVector(45.25, 270, 0.7, 0, -90, true);
            turnHeading(0.3, 35);
            lColl.setPower(1);
            rColl.setPower(1);
            driveVectorRot(27, 0, 0.4, 0, -35 , false);
            turnHeading(0.4, 0);
            lColl.setPower(0);
            rColl.setPower(0);
            driveVector(10.5, 180, 0.4, 0, 0, false);

            turnHeading(0.3, -90);
            driveVector(69, 90, 0.7, 0, -90, true);
            setCollect(-0.6);

            waitSec(1);
            setCollect(0);
            lColl.setPower(0);
            rColl.setPower(0);

            driveVectorRot(8, 270, 0.7, 0, -90, false);

        } else if (scan == SkystonePosition.CENTER){

            telemetry.addData("Position: ", "R");
            telemetry.update();

            driveVector(5.15, 90, 0.6, 0, 0, true);
            turnHeading(0.3, 35);
            lColl.setPower(1);
            rColl.setPower(1);
            driveVectorRot(30, 0, 0.4, 0, -35 , false);
            turnHeading(0.3, 0);
            lColl.setPower(0);
            rColl.setPower(0);
            driveVector(17, 180, 0.3, 0, 0, false);


            turnHeading(0.3, -90);
            driveVector(43, 90, 0.7, 0, -90, true);
            setCollect(-0.6);

            waitSec(1);
            setCollect(0);
            lColl.setPower(0);
            rColl.setPower(0);

            driveVector(42.75, 270, 0.7, 0, -90, true);
            turnHeading(0.3, 35);
            lColl.setPower(1);
            rColl.setPower(1);
            driveVectorRot(27, 0, 0.4, 0, -35 , false);
            turnHeading(0.4, 0);
            lColl.setPower(0);
            rColl.setPower(0);
            driveVector(12, 180, 0.4, 0, 0, false);

            turnHeading(0.3, -90);
            driveVector(64, 90, 0.7, 0, -90, true);
            setCollect(-0.6);

            waitSec(1);
            setCollect(0);
            lColl.setPower(0);
            rColl.setPower(0);

            driveVectorRot(8, 270, 0.7, 0, -90, false);

        } else {

            telemetry.addData("Position: ", "L");
            telemetry.update();

            driveVector(4.55, 270, 0.6, 0, 0, true);
            turnHeading(0.3, 35);
            lColl.setPower(1);
            rColl.setPower(1);
            driveVectorRot(30, 0, 0.4, 0, -35 , false);
            turnHeading(0.3, 0);
            lColl.setPower(0);
            rColl.setPower(0);
            driveVector(17, 180, 0.3, 0, 0, false);


            turnHeading(0.3, -90);
            driveVector(54, 90, 0.7, 0, -90, true);
            setCollect(-0.6);



            waitSec(2);
            setCollect(0);
            lColl.setPower(0);
            rColl.setPower(0);



            driveVector(49.25, 270, 0.7, 0, -90, true);
            turnHeading(0.3, 35);
            lColl.setPower(1);
            rColl.setPower(1);
            driveVectorRot(27, 0, 0.4, 0, -35 , false);
            turnHeading(0.4, 0);
            lColl.setPower(0);
            rColl.setPower(0);
            driveVector(10.5, 180, 0.4, 0, 0, false);

            turnHeading(0.3, -90);
            driveVector(69, 90, 0.7, 0, -90, true);
            setCollect(-0.6);

            waitSec(1);
            setCollect(0);
            lColl.setPower(0);
            rColl.setPower(0);
          
          

            driveVectorRot(8, 270, 0.7, 0, -90, false);

        }

        stop();

    }

}
