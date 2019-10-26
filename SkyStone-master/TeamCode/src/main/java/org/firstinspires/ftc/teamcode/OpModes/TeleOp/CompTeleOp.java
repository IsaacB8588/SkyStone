package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.RobotRunType;

@TeleOp(name = "TeleOp Test")
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
            if (gamepad2.right_bumper){
                lColl.setPower(1);
                rColl.setPower(1);
                lfeeder.setPosition(1);
                rfeeder.setPosition(0);

            //reverse collector with left bumper
            } else if (gamepad2.left_bumper){
                lColl.setPower(-1);
                rColl.setPower(-1);
                lfeeder.setPosition(0);
                rfeeder.setPosition(1);

            } else {
                lColl.setPower(0);
                rColl.setPower(0);
                lfeeder.setPosition(0.5);
                rfeeder.setPosition(0.5);

            }

            //use down on dpad to quickly collapse the elevator
            if (gamepad2.dpad_down){
                if (spool.getCurrentPosition() > 15){
                    if (spool.getCurrentPosition() > 130){
                        spool.setPower(-1);
                    } else {
                        spool.setPower(-0.2);
                    }
                } else {
                    spool.setPower(0);
                }
            } else {
                //use left joystick to run the elevator
                spool.setPower(-gamepad2.left_stick_y);
            }

            if (gamepad2.x){
                grabCount++;
            } else {
                grabCount = 0;
            }

            if (gamepad2.a){
                flipCount++;
            } else {
                flipCount = 0;
            }

            if (grabCount == 1){
                grabbing = !grabbing;
                if (!grabbing){
                    grab.setPosition(0);
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

        }
        stop();
    }

}
