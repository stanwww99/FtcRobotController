package org.firstinspires.ftc.teamcode.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import android.graphics.Color;

@Autonomous(name = "ColorSensorTest", group = "Autonomous")
public class ColorSensorTest extends LinearOpMode {

    // FTC color sensor interface that returns normalized RGBA values
    private NormalizedColorSensor colorSensor;

    @Override
    public void runOpMode() {

        // Retrieve the color sensor from the Robot Configuration.
        // The name "colorSensor" must match the configuration on the Driver Station.
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        // Initial telemetry before the OpMode starts
        telemetry.clear();
        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();

        // Wait for PLAY button
        waitForStart();

        // Main loop: continuously read color and classify it
        while (opModeIsActive()) {

            // Determine which color is detected based on hue ranges
            if (isColorGreen()) {
                telemetry.addData("Detected Color", "green");
            } else if (isColorPurple()) {
                telemetry.addData("Detected Color", "purple");
            } else {
                telemetry.addData("Detected Color", "not green and not purple");
            }

            // Display raw hue value for debugging and calibration
            telemetry.addData("Hue", "%.1f", readColor()[0]);
            telemetry.update();
        }
    }

    /**
     * Reads the current color from the sensor and converts it to HSV.
     * HSV is more stable for color classification than raw RGB.
     *
     * @return float[] containing HSV values:
     *         hsv[0] = hue (0–360 degrees)
     *         hsv[1] = saturation
     *         hsv[2] = value (brightness)
     */
    private float[] readColor() {

        // Get normalized RGBA values from the sensor
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        // Prepare HSV array
        float[] hsv = new float[3];

        // Convert the RGBA color into HSV using Android's built-in utility
        Color.colorToHSV(colors.toColor(), hsv);

        return hsv;
    }

    /**
     * Determines whether the detected hue falls within the "green" range.
     * FTC color sensors often produce hue values around 120–180 for green objects.
     */
    private boolean isColorGreen() {
        float[] hsv = readColor();
        double hue = hsv[0];
        return (hue >= 120 && hue <= 180);
    }

    /**
     * Determines whether the detected hue falls within the "purple" range.
     * Purple typically appears around 181–245 degrees in HSV hue space.
     */
    private boolean isColorPurple() {
        float[] hsv = readColor();
        double hue = hsv[0];
        return (hue >= 181 && hue <= 245);
    }
}
