package org.firstinspires.ftc.teamcode.year20252026.opModes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import java.util.Arrays;


@Autonomous(name= "ColorCalibration", group = "Autonomous")
public class ColorCalibration extends LinearOpMode {
    NormalizedColorSensor colorSensorFront;
    NormalizedColorSensor colorSensorBack;
    public static float[] hsv1;
    public static float[] hsv2;

    public void runOpMode() throws InterruptedException {
        initColorSensors();
        telemetry.clear();
        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()){
            readColor();
            updateTelemetry();
        }

    }
    public void updateTelemetry(){
        telemetry.clear();
        telemetry.addData("Color Sensor 1 HSV", Arrays.toString(hsv1));
        telemetry.addData("Color Sensor 2 HSV", Arrays.toString(hsv2));
        telemetry.update();
    }
    public void initColorSensors(){
        colorSensorFront = hardwareMap.get(NormalizedColorSensor.class, "colorSensorFront");
        colorSensorBack = hardwareMap.get(NormalizedColorSensor.class, "colorSensorBack");
    }
    private void readColor() {
        // Read normalized RGBA values
        NormalizedRGBA colorsfront = colorSensorFront.getNormalizedColors();
        hsv1 = new float[3];
        // Convert to HSV
        Color.colorToHSV(colorsfront.toColor(), hsv1);
        NormalizedRGBA colorsBack = colorSensorBack.getNormalizedColors();
        hsv2 = new float[3];
        // Convert to HSV
        Color.colorToHSV(colorsBack.toColor(), hsv2);

    }
}
