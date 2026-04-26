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

    // Two independent color sensors mounted on the robot.
    // Used for calibration and comparison of front/back readings.
    NormalizedColorSensor colorSensorFront;
    NormalizedColorSensor colorSensorBack;

    // HSV arrays storing the converted color values from each sensor.
    // HSV is more stable for classification than raw RGB.
    public static float[] hsv1;
    public static float[] hsv2;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize both sensors from the hardware map.
        initColorSensors();

        telemetry.clear();
        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();

        // Wait for PLAY button.
        waitForStart();

        // Main loop: continuously read and display HSV values.
        while(opModeIsActive()){
            readColor();       // Convert both sensors' readings to HSV
            updateTelemetry(); // Display results for calibration
        }
    }

    /**
     * Updates telemetry with the latest HSV values from both sensors.
     * Arrays.toString() is used for easy debugging and calibration.
     */
    public void updateTelemetry(){
        telemetry.clear();
        telemetry.addData("Color Sensor 1 HSV", Arrays.toString(hsv1));
        telemetry.addData("Color Sensor 2 HSV", Arrays.toString(hsv2));
        telemetry.update();
    }

    /**
     * Initializes both color sensors.
     * Names must match the Robot Configuration in the FTC Driver Station.
     */
    public void initColorSensors(){
        colorSensorFront = hardwareMap.get(NormalizedColorSensor.class, "colorSensorFront");
        colorSensorBack  = hardwareMap.get(NormalizedColorSensor.class, "colorSensorBack");
    }

    /**
     * Reads normalized RGBA values from both sensors and converts them to HSV.
     *
     * Why HSV?
     * - Hue (0–360°) is extremely useful for distinguishing colors.
     * - Saturation and Value help detect lighting conditions.
     * - HSV is more stable than raw RGB under different lighting.
     */
    private void readColor() {

        // --- FRONT SENSOR ---
        NormalizedRGBA colorsFront = colorSensorFront.getNormalizedColors();
        hsv1 = new float[3];
        Color.colorToHSV(colorsFront.toColor(), hsv1);

        // --- BACK SENSOR ---
        NormalizedRGBA colorsBack = colorSensorBack.getNormalizedColors();
        hsv2 = new float[3];
        Color.colorToHSV(colorsBack.toColor(), hsv2);
    }
}
