package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.RobotRunType;

@TeleOp(name = "Competition TeleOp")
public class CompTeleOp extends RobotHardware {

    public void runOpMode(){

        initRobot(RobotRunType.TELEOP);

        boolean grabbing = false;
        int grabCount = 0;
        boolean flipped = false;
        int flipCount = 0;

        boolean stoneGrabBoolean = false;
        int stoneGrabCount = 0;
        boolean stoneArmBoolean = false;
        int stoneArmCount = 0;

        int upCount = 0;
        int downCount = 0;
        int level = 0;

        boolean fullUp = false;
        int fullUpCount = 0;

        boolean last = false;

        int count = 0;

        final double ppr = ORBITAL20_PPR;
        final double spoolDiameter = 2;
        final double spoolCirc = Math.PI * spoolDiameter;
        final double ppi = ppr/spoolCirc;

        spool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(spool.getMode() != DcMotor.RunMode.STOP_AND_RESET_ENCODER){}
        spool.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        spool.setPower(1);

        boolean clear = false;
        boolean returning = false;
        boolean raising = false;
        boolean dropped = true;
        boolean smellCap = false;

        int speed = 45;

        waitForStart();
        while(opModeIsActive()) {

            /** //////////////////////////
             *  /////////driver 1/////////
             *  //////////////////////////
             */

            //////////////////////////
            ////drivetrain control////
            //////////////////////////

            //call Field-Centric Drive Method
            FieldCentricDrive();

            //use a + b to reset the robot heading for drive
            if (gamepad1.a & gamepad1.b) {
                globalAngle = 0;
            }

            //////////////////////////
            /////foundation hooks/////
            //////////////////////////

            if (gamepad1.right_bumper) {
                hookL.setPosition(0);
                hookR.setPosition(1);
            } else if (gamepad1.left_bumper) {
                hookL.setPosition(0.3);
                hookR.setPosition(0.85);
            }

            //////////////////////////
            /////////stone arm////////
            //////////////////////////

            if (gamepad1.x) {
                fullUpCount++;
            } else {
                fullUpCount = 0;
            }

            if (fullUpCount == 1) {
                fullUp = !fullUp;
            }

            if (gamepad1.dpad_up) {
                stoneArmCount++;
            } else {
                stoneArmCount = 0;
            }

            if (stoneArmCount == 1) {
                stoneArmBoolean = !stoneArmBoolean;
                if (!stoneArmBoolean) {
                    if (fullUp) {
                        stoneArm.setPosition(1);
                    } else {
                        stoneArm.setPosition(0.5);
                    }
                } else {
                    stoneArm.setPosition(0.1);
                }
            }

            if (gamepad1.dpad_down) {
                stoneGrabCount++;
            } else {
                stoneGrabCount = 0;
            }

            if (stoneGrabCount == 1) {
                stoneGrabBoolean = !stoneGrabBoolean;
                if (!stoneGrabBoolean) {
                    stoneGrab.setPosition(0);
                } else {
                    stoneGrab.setPosition(1);
                }
            }


            /** //////////////////////////
             *  /////////driver 2/////////
             *  //////////////////////////
             */

            //////////////////////////
            /////////collector////////
            //////////////////////////

            //run collector with right and left triggers
            double power = 0.55*(-gamepad2.right_trigger+gamepad2.left_trigger);
            lColl.setPower(power);
            rColl.setPower(power);

            if (gamepad2.right_trigger > 0.05 && spool.getCurrentPosition() < 5*ppi){
                raiseArm(-1);
                last = true;
            } else {
                if (last){
                    last = false;
                    raiseArm(0);
                }
            }

            //////////////////////////
            ////level adjustments/////
            //////////////////////////

            if (gamepad2.dpad_up) {
                upCount++;
            } else {
                upCount = 0;
            }

            if (upCount == 1 && level != 11) {
                level++;
            }

            if (gamepad2.dpad_down) {
                downCount++;
            } else {
                downCount = 0;
            }

            if (downCount == 1 && level != 0) {
                level--;
            }

            if (smellCap){
                speed = 15;
            } else {
                speed = 35;
            }

            if(-gamepad2.left_stick_y > 0){
                spool.setTargetPosition(spool.getTargetPosition() + speed);
            } else if (-gamepad2.left_stick_y < 0){
                spool.setTargetPosition(spool.getTargetPosition() - speed);
            }

            //////////////////////////
            //////manual controls/////
            //////////////////////////

            if (gamepad2.a) {
                grabCount++;
            } else {
                grabCount = 0;
            }

            if (gamepad2.x) {
                flipCount++;
            } else {
                flipCount = 0;
            }

            if (grabCount == 1) {
                grabbing = !grabbing;
                if (!grabbing) {
                    grab.setPosition(0.3);
                    if (flipped && level < 10){
                        level++;
                    }
                } else {
                    grab.setPosition(1);
                }
            }

            if (flipCount == 1) {
                flipped = !flipped;
                if (!flipped) {
                    flip.setPosition(0);
                } else {
                    flip.setPosition(1);
                }

            }

            //////////////////////////
            ////////auto-extend///////
            //////////////////////////

            if (gamepad2.right_bumper){
                if (!raising){
                    if (!smellCap){
                        spool.setPower(1);
                    }

                    raising = true;
                    if (level < 3){
                        spool.setTargetPosition((int)(13*ppi));
                        dropped = false;
                    } else {
                        raiseArm(level);
                        dropped = true;
                    }
                    clear = false;
                }
            }

            if (raising){
                if(!flipped){
                    if (!clear){
                        if (level < 3){
                            if (spool.getCurrentPosition() > 12*ppi){
                                clear = true;
                            }
                        } else {
                            if (spool.getCurrentPosition() > 4*level*ppi){
                                clear = true;
                            }
                        }
                    } else {

                        if (!smellCap){
                            flip.setPosition(1);
                            count++;
                            if (count > 20){
                                flipped = true;
                            }
                        } else {
                            count = 0;
                            raising = false;
                        }

                    }
                } else {
                    if (!dropped){
                        spool.setPower(0.5);
                        raiseArm(level);
                    }
                    raising = false;
                    count = 0;
                }
            }

            //////////////////////////
            ////////auto-return///////
            //////////////////////////

            if (gamepad2.left_bumper && flipped){
                if (!returning){
                    spool.setPower(1);
                    returning = true;
                    if (spool.getCurrentPosition() < 12*ppi){
                        spool.setTargetPosition((int) (13*ppi));
                    } else {
                        raiseArm(level);
                    }
                    clear = false;
                }
            } else if (gamepad2.left_bumper)  {
                if (!returning){
                    spool.setPower(1);
                    raiseArm(0);
                }
            }

            if (returning){
                if (flipped){
                    if (!clear){

                        if (level < 3){
                            if (spool.getCurrentPosition() > 12*ppi){
                                clear = true;
                            }
                        } else {
                            if (spool.getCurrentPosition() > 4*(level - 1)*ppi + 2*ppi){
                                clear = true;
                            }
                        }
                    } else {
                        flip.setPosition(0);
                        count++;
                        if (count > 20){
                            flipped = false;
                        }
                    }
                } else {
                    raiseArm(0);
                    returning = false;
                    count = 0;
                }
            }

            //////////////////////////
            /////////capstone/////////
            //////////////////////////

            if (gamepad2.right_stick_button) {
                noCap.setPosition(0);
                spool.setPower(0.5);
                smellCap = true;
            } else if (gamepad2.left_stick_button) {
                noCap.setPosition(1);
                spool.setPower(1);
                smellCap = false;
            }

            /** //////////////////////////
             *  ////////telemetry/////////
             *  //////////////////////////
             */

            telemetry.addData("Heading: ", getGlobal() % 360);
            telemetry.addData("Arm Full Up: ", fullUp);
            telemetry.addData("Level: ", level + 1);
            telemetry.addData("Arm Height: ", spool.getCurrentPosition()/ppi);
            telemetry.update();

        }
        stop();
    }

}
