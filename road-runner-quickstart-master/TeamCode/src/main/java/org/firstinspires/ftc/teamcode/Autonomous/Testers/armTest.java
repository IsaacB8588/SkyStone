package org.firstinspires.ftc.teamcode.Autonomous.Testers;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.RobotRunType;

@TeleOp(name = "arm")
@Disabled
public class armTest extends RobotHardware {

    @Override
    public void runOpMode(){

        initRobot(RobotRunType.TELEOP);


        int upCount = 0;
        int downCount = 0;
        int level = 0;

        boolean grabbing = false;
        int grabCount = 0;
        boolean flipped = false;
        int flipCount = 0;

        spool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(spool.getMode() != DcMotor.RunMode.STOP_AND_RESET_ENCODER){

        }
        spool.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        spool.setPower(1);

        waitForStart();

        while (opModeIsActive()){

            if (gamepad2.dpad_up){
                upCount++;
            } else {
                upCount = 0;
            }

            if (upCount == 1 && level != 11){
                level++;
            }

            if (gamepad2.dpad_down){
                downCount++;
            } else {
                downCount = 0;
            }

            if (downCount == 1 && level != 0){
                level--;
            }

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

            if (gamepad2.y){
                raiseArm(level);
            } else if (gamepad2.b){
                raiseArm(0);
            }

        }
        stop();

    }

}
