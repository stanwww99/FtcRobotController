package org.firstinspires.ftc.teamcode.year20252026.calibration;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Set0DegreesServoMotor", group = "Autonomous")
public class Set0DegreesServoMotor extends LinearOpMode {

    // Reference to the servo object
    private Servo servo;

    // Many FTC servos support ~300 degrees of rotation (for positional servos)
    private static final double SERVO_FULL_RANGE_DEG = 300;

    @Override
    public void runOpMode() {

        // Retrieve servo from the Robot Configuration (must match the config name)
        servo = hardwareMap.get(Servo.class, "pusher");

        // Display initial servo position before starting
        telemetry.clearAll();
        telemetry.addLine("Ready. Press Play to start.");
        telemetry.addData("Servo Position", servo.getPosition());
        telemetry.update();

        // Wait for driver to press PLAY
        waitForStart();

        // Move servo to 0 degrees (mapped to position 0.0)
        setServoAngle(servo, 0);

        // Display updated servo position
        telemetry.clearAll();
        telemetry.addLine("Servo moved to 0 degrees.");
        telemetry.addData("Servo Position", servo.getPosition());
        telemetry.update();

        // Keep OpMode alive so the servo holds position until STOP is pressed
        while (opModeIsActive()) {
            // No actions needed; servo holds its last commanded position
        }
    }

    /**
     * Converts an angle in degrees to a servo position (0.0 to 1.0)
     * and sends it to the servo.
     *
     * @param s         The servo to control
     * @param angleDeg  Desired angle in degrees (0–300)
     */
    private void setServoAngle(Servo s, double angleDeg) {
        // Convert degrees → normalized servo position
        double pos = RangeClip(angleDeg / SERVO_FULL_RANGE_DEG, 0.0, 1.0);
        s.setPosition(pos);
    }

    /**
     * Restricts a value to a given range.
     * Equivalent to Range.clip() but implemented manually.
     *
     * @param v    Value to clip
     * @param min  Minimum allowed value
     * @param max  Maximum allowed value
     * @return     Clipped value
     */
    private double RangeClip(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
