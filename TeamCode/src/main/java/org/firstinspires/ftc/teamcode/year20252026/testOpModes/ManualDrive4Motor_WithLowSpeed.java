package org.firstinspires.ftc.teamcode.year20252026.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp mode for a standard 4‑motor mecanum drivetrain.
 * Includes a "low‑speed mode" activated by the right trigger,
 * allowing drivers to make precise movements at 30% max power.
 *
 * This file demonstrates:
 *  - Hardware mapping of motors
 *  - Correct motor direction configuration for mecanum
 *  - Power normalization and clipping
 *  - Real‑time joystick reading and mixing
 *  - Runtime telemetry for debugging
 */
@TeleOp(name = "ManualDrive4Motor_WithLowSpeed", group = "TeleOp")
public class ManualDrive4Motor_WithLowSpeed extends LinearOpMode {

    // Motor objects — FTC SDK creates these as interfaces to the REV Hub motor ports.
    private DcMotor leftFront = null;
    private DcMotor leftBack  = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack  = null;

    // --- CONFIGURATION CONSTANTS ---
    // Max output when low‑speed mode is active (right trigger pressed)
    private static final double LOW_SPEED_MAX = 0.30; // 30% power cap
    // Max output when full‑speed mode is active
    private static final double FULL_SPEED_MAX = 1.0; // 100% power
    // Deadband to ignore tiny joystick noise (not used in your current code but kept for clarity)
    private static final double JOYSTICK_DEADBAND = 0.05;

    @Override
    public void runOpMode() {

        // --- HARDWARE MAPPING ---
        // hardwareMap.get() retrieves motor objects by name from the robot configuration.
        // If names do not match the configuration, the OpMode will crash on init.
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        // --- MOTOR DIRECTION SETUP ---
        // Mecanum wheels require correct motor direction to ensure:
        //  - Forward motion pushes all wheels forward
        //  - Strafing left/right behaves correctly
        //  - Rotation behaves symmetrically
        //
        // Typically, the right side motors must be reversed because they are mirrored.
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // --- ZERO POWER BEHAVIOR ---
        // BRAKE mode actively resists motion when power = 0.
        // This improves precision during slow maneuvers and prevents drifting.
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.clearAll();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for driver to press PLAY on the Driver Station.
        waitForStart();

        // --- MAIN CONTROL LOOP ---
        // Runs continuously until STOP is pressed.
        while (opModeIsActive()) {

            // Handle joystick input and motor power output.
            drive();

            // Telemetry shows real‑time motor power values.
            telemetry.addData("LF",  leftFront.getPower());
            telemetry.addData("LB",  leftBack.getPower());
            telemetry.addData("RF", rightFront.getPower());
            telemetry.addData("RB", rightBack.getPower());
            telemetry.update();
        }
    }

    /**
     * Core drive logic for mecanum movement.
     * Reads joystick input, mixes into mecanum wheel powers,
     * applies speed limiting, and sends output to motors.
     */
    private void drive(){

        // --- READ GAMEPAD INPUT ---
        // FTC SDK updates gamepad values every loop.
        // left_stick_x  → strafing left/right
        // left_stick_y  → forward/back (inverted because pushing forward gives negative)
        // right_stick_x → rotation left/right
        double lx = gamepad1.left_stick_x;
        double ly = -gamepad1.left_stick_y;   // invert to make forward = positive
        double rx = -gamepad1.right_stick_x;  // invert to make clockwise rotation positive

        // --- MECANUM DRIVE MIXING ---
        // Standard mecanum formula:
        //   lf = forward + strafe + rotate
        //   rf = forward - strafe - rotate
        //   lb = forward - strafe + rotate
        //   rb = forward + strafe - rotate
        //
        // This ensures:
        //  - Strafing uses opposite diagonal wheels
        //  - Rotation uses opposite sides
        //  - Forward/back uses all wheels equally
        double lf = ly + lx + rx;
        double rf = ly - lx - rx;
        double lb = ly - lx + rx;
        double rb = ly + lx - rx;

        // --- NORMALIZATION / CLIPPING ---
        // Ensures motor power stays within [-1, 1].
        // Your custom rangeClip() is equivalent to Range.clip().
        lf = rangeClip(-lf, -1.0, 1.0);
        rf = rangeClip(-rf, -1.0, 1.0);
        lb = rangeClip(-lb, -1.0, 1.0);
        rb = rangeClip(-rb, -1.0, 1.0);

        // --- LOW SPEED MODE ---
        // If right trigger is pressed even slightly (>0.05),
        // limit all motor power to 30% for precision driving.
        double speedLimit = gamepad1.right_trigger > 0.05 ? LOW_SPEED_MAX : FULL_SPEED_MAX;

        // --- APPLY MOTOR POWER ---
        // Multiply each wheel's computed power by the speed limit.
        leftFront.setPower(lf * speedLimit);
        rightFront.setPower(rf * speedLimit);
        leftBack.setPower(lb * speedLimit);
        rightBack.setPower(rb * speedLimit);
    }

    /**
     * Clips a value between min and max.
     * Equivalent to Range.clip(), but implemented manually.
     */
    private double rangeClip(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
