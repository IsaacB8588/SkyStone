package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@Disabled
@TeleOp(name = "color test")
public class ColorSensorTest extends LinearOpMode {

    DistanceSensor distance;
    ColorSensor color;

    public void runOpMode(){

        color = hardwareMap.get(ColorSensor.class, "color");
        distance = hardwareMap.get(DistanceSensor.class, "color");

        waitForStart();
        while(opModeIsActive()){

            telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", distance.getDistance(DistanceUnit.CM)));
            telemetry.addData("Red  ", color.red());
            telemetry.addData("Green", color.green());
            telemetry.addData("Blue ", color.blue());

            if (color.blue() != 0){
                if (color.red()/color.blue() > 1.6){
                    telemetry.addData("Skystone", false);
                } else {
                    telemetry.addData("Skystone", true);

                }
            }

            telemetry.update();

        }


    }

}
