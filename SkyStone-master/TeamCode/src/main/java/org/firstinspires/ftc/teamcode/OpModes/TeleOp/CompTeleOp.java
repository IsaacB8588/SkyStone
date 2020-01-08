package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.RobotRunType;

@TeleOp(name = "Competition TeleOp")
public class CompTeleOp extends RobotHardware {

    public void runOpMode(){

        initRobot(RobotRunType.TELEOP);
        spool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spool.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        boolean grabbing = false;
        int grabCount = 0;
        boolean flipped = false;
        int flipCount = 0;

        waitForStart();
        while(opModeIsActive()){

            //////////////////////////
            /////////driver 1/////////
            //////////////////////////

            //call Field-Centric Drive Method
            FieldCentricDrive();

            //use a + b to reset the robot heading for drive
            if (gamepad1.a & gamepad1.b){
                globalAngle = 0;
            }

            //////////////////////////
            /////////driver 2/////////
            //////////////////////////

            //run collector with right bumper
            if (gamepad2.left_bumper){
                lColl.setPower(0.55);
                rColl.setPower(0.55);

            //reverse collector with left bumper
            } else if (gamepad2.right_bumper){
                lColl.setPower(-0.55);
                rColl.setPower(-0.55);

            } else {
                lColl.setPower(0);
                rColl.setPower(0);

            }


            //use left joystick to run the elevator
            spool.setPower(-gamepad2.left_stick_y);



            if (gamepad2.a){
                grabCount++;
            } else {
                grabCount = 0;
            }

            if (gamepad2.x){
                flipCount++;
            } else {
                flipCount = 0;
            }

            if (grabCount == 1){
                grabbing = !grabbing;
                if (!grabbing){
                    grab.setPosition(0.3);
                } else {
                    grab.setPosition(1);
                }
            }

            if (flipCount == 1){
                flipped = !flipped;
                if (!flipped){
                    flip.setPosition(1);
                } else {
                    flip.setPosition(0);
                }

            }

            if(gamepad2.dpad_up){
                noCap.setPosition(0);
            } else if (gamepad2.dpad_down){
                noCap.setPosition(1);
            }

            if (gamepad1.right_bumper){
                hookL.setPosition(1);
                hookR.setPosition(0);
            } else if (gamepad1.left_bumper) {
                hookL.setPosition(0);
                hookR.setPosition(1);
            }

            telemetry.addData("current", getGlobal() % 360);
            telemetry.update();

        }
        stop();
    }

}
