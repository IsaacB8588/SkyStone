package org.firstinspires.ftc.teamcode.ExampleBots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "adams POS")
public class Adam extends LinearOpMode {

    DcMotor isaacIsDumbR;
    DcMotor isaacIsDumbL;
    DcMotor arm;
    Servo flip;

    public void runOpMode(){
        //after init

        isaacIsDumbR = hardwareMap.dcMotor.get("rd");
        isaacIsDumbL = hardwareMap.dcMotor.get("ld");
        isaacIsDumbL.setDirection(DcMotorSimple.Direction.REVERSE);

        arm = hardwareMap.dcMotor.get("arm");

        flip = hardwareMap.servo.get("flip");

        waitForStart();
        //after start

        while (opModeIsActive()){
            isaacIsDumbR.setPower(-gamepad1.right_stick_y);

            isaacIsDumbL.setPower(-gamepad1.left_stick_y);

            if (gamepad1.a){
                arm.setPower(1);
            }
            else if (gamepad1.b){
                arm.setPower(-1);
            }
            else {
                arm.setPower(0);
            }

            if (gamepad1.x){
                flip.setPosition(0);
            } else if (gamepad1.y){
                flip.setPosition(1);
            }

        }

        stop();

    }

}
