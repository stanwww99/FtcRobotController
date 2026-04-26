package org.firstinspires.ftc.teamcode.year20252026.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="ManualDrive2Motor_WithLowSpeed", group="TeleOp")
public class ManualDrive2Motor_WithLowSpeed extends LinearOpMode {

    // Two‑motor drivetrain (tank-style or simple differential)
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    // Tracks whether low-speed mode is currently active
    private boolean lowSpeedMode = false;

    @Override
    public void runOpMode() {

        // --- HARDWARE INITIALIZATION ---
        // Retrieve motors from the Robot Configuration.
        // Names must match exactly what is set in the FTC Driver Station config.
        leftMotor  = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        // Motor direction correction:
        // Most drivetrains require reversing one side so that
        // "positive power" makes both wheels spin forward.
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Wait for driver to press PLAY
        waitForStart();

        // --- MAIN CONTROL LOOP ---
        while (opModeIsActive()) {

            // Handle joystick → motor power logic
            drive();

            // --- TELEMETRY ---
            telemetry.addData("Left Power", leftMotor.getPower());
            telemetry.addData("Right Power", rightMotor.getPower());
            telemetry.addData("Low Speed Mode", lowSpeedMode);
            telemetry.update();
        }
    }

    /**
     * Clips a value to a given range.
     * Equivalent to Range.clip(), but implemented manually.
     */
    private double rangeClip(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    /**
     * Reads joystick inputs, computes motor powers,
     * applies low-speed mode scaling, and sends power to motors.
     */
    private void drive() {

        // --- JOYSTICK INPUTS ---
        // left_stick_y: forward/backward movement
        // FTC gamepad Y-axis is inverted, so we negate it.
        double driveY = -gamepad1.left_stick_y;

        // left_stick_x: strafing (only meaningful on mecanum or omni wheels)
        // On a 2-motor drivetrain, this will cause asymmetric power distribution.
        double strafeX = gamepad1.left_stick_x;

        // right_stick_x: rotation/spin in place
        double spinX = gamepad1.right_stick_x;

        // --- LOW SPEED MODE ---
        // Activated when right trigger is pressed beyond a small threshold.
        // Useful for precision driving near scoring areas.
        lowSpeedMode = gamepad1.right_trigger > 0.05;

        // Scale motor output:
        // 1.0 = full speed
        // 0.3 = slow mode (30% power)
        double speedScale = lowSpeedMode ? 0.3 : 1.0;

        // --- MOTOR POWER MIXING ---
        // Combine forward/back + strafe + rotation.
        // This formula is normally used for mecanum drive,
        // but here it simply produces differential steering behavior.
        double leftPower  = driveY + strafeX - spinX;
        double rightPower = driveY - strafeX + spinX;

        // Ensure values stay within [-1, 1]
        leftPower  = rangeClip(leftPower,  -1, 1);
        rightPower = rangeClip(rightPower, -1, 1);

        // --- APPLY POWER TO MOTORS ---
        leftMotor.setPower(leftPower * speedScale);
        rightMotor.setPower(rightPower * speedScale);
    }
}
