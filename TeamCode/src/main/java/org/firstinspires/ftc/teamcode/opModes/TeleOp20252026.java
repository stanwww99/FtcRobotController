package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOp20252026", group = "TeleOp")
public class TeleOp20252026 extends LinearOpMode {

    // Drive motors (Control Hub)
    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;

    // Expansion Hub motors
    private DcMotorEx carousel;    // GoBilda 60 RPM gearbox motor (use encoder)
    private DcMotorEx intake;      // Tetrix
    private DcMotorEx shooter;     // GoBilda 6000 rpm motor with encoder

    // Servo
    private Servo pusher;          // multimode smart servo, initialize to 0 degrees

    // Shooter RPM tracking variables (requested)
    private double currentRPM = 0.0;
    private double targetRPM = 0.0;
    private boolean targetMet = false;

    // Configuration / constants (adjust to match actual encoders)
    // Assumptions you may want to tune:
    // - MOTOR_TICKS_PER_REV: encoder counts per motor shaft revolution (typical GoBilda = 28 CPR)
    // - CAROUSEL_GEAR_REDUCTION: gearbox ratio for carousel output (e.g., 99.5 for the 60 rpm gearbox)
    private static final double MOTOR_TICKS_PER_REV = 28.0;        // adjust if your encoder differs
    private static final double CAROUSEL_GEAR_REDUCTION = 99.5;    // gearbox ratio from reference
    private static final double CAROUSEL_COUNTS_PER_OUTPUT_REV = MOTOR_TICKS_PER_REV * CAROUSEL_GEAR_REDUCTION;

    // Shooter encoder counts per output revolution (if direct drive use MOTOR_TICKS_PER_REV)
    private static final double SHOOTER_TICKS_PER_REV = MOTOR_TICKS_PER_REV;

    // Servo angles -> servo.setPosition expects 0..1 mapping; we'll map 0..300 degrees (±150) to 0..1
    private static final double SERVO_RANGE_DEGREES = 300.0;       // servo multimode standard mode ±150 = 300° range
    private static final double SERVO_NEUTRAL_DEG = 0.0;          // start at 0°
    private static final double SERVO_TO_POSITION_FACTOR = 1.0 / SERVO_RANGE_DEGREES;

    @Override
    public void runOpMode() throws InterruptedException {
        // hardware map names must match those you provided
        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotorEx.class, "rightBack");

        carousel = hardwareMap.get(DcMotorEx.class, "carousel");
        intake   = hardwareMap.get(DcMotorEx.class, "intake");
        shooter  = hardwareMap.get(DcMotorEx.class, "shooter");

        pusher = hardwareMap.get(Servo.class, "pusher");

        // Motor directions: adjust if your robot moves inverted
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // Use encoders where needed
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize servo to 0 degrees
        setServoAngleDegrees(pusher, 0.0);

        // Initialize carousel to 0 degrees (assume encoder position 0 is 0°)
        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        carousel.setTargetPosition(0);
        carousel.setPower(0.0);
        // After reset, set to RUN_USING_ENCODER for manual position moves
        carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- Driving controls (gamepad1 sticks) ---
            float lf, rf, lb, rb;
            float speedScale = gamepad1.right_trigger > 0.01 ? 0.3f : 1.0f; // RT pressed => low speed 30%

            // Left stick controls strafing + forward/back
            float driveY = -gamepad1.left_stick_y;   // push up => negative, invert so up = positive forward
            float driveX = gamepad1.left_stick_x;    // left negative -> strafe left

            // Right stick controls rotation
            float rotate = gamepad1.right_stick_x;   // left negative => CCW, right positive => CW

            // Combine for mecanum-style control (assuming mecanum or mecanum-like holonomic)
            // We'll compute wheel powers: standard mix for mecanum
            float frontLeftPower  = driveY + driveX + rotate;
            float frontRightPower = driveY - driveX - rotate;
            float backLeftPower   = driveY - driveX + rotate;
            float backRightPower  = driveY + driveX - rotate;

            // Normalize powers
            float max = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            // Apply speed scaling
            frontLeftPower  *= speedScale;
            frontRightPower *= speedScale;
            backLeftPower   *= speedScale;
            backRightPower  *= speedScale;

            leftFront.setPower(frontLeftPower);
            rightFront.setPower(frontRightPower);
            leftBack.setPower(backLeftPower);
            rightBack.setPower(backRightPower);

            // --- Intake Motor (gamepad1 LB and LT; A limits max to 30%) ---
            double intakePower = 0.0;
            double intakeLimit = gamepad1.a ? 0.3 : 1.0; // if A held, limit to 30%

            if (gamepad1.left_bumper) {
                // LB -> go backwards at full (or limited) power
                intakePower = -1.0 * intakeLimit;
            } else if (gamepad1.left_trigger > 0.01) {
                // LT analog controls forward speed
                double requested = gamepad1.left_trigger; // 0..1
                intakePower = Range.clip(requested, -1.0, 1.0) * intakeLimit;
            } else {
                intakePower = 0.0;
            }
            intake.setPower(intakePower);

            // --- Shooter Motor (gamepad2 X/A/B/Y) ---
            // Variables currentRPM, targetRPM, targetMet must update after motor change
            if (gamepad2.x) {
                // target 1000 RPM
                setShooterTargetRPM(1000.0);
            }
            if (gamepad2.a) {
                // target 2500 RPM
                setShooterTargetRPM(2500.0);
            }
            if (gamepad2.b) {
                // target 5000 RPM
                setShooterTargetRPM(5000.0);
            }
            if (gamepad2.y) {
                // stop
                setShooterTargetRPM(0.0);
            }

            // Read actual shooter RPM from encoder velocity
            // DcMotorEx.getVelocity() returns ticks per second (when supported); convert to RPM:
            double ticksPerSecond = shooter.getVelocity(); // ticks per second
            currentRPM = (ticksPerSecond * 60.0) / SHOOTER_TICKS_PER_REV;

            // Update targetMet according to rule: targetRPM is "desired target minimum" defined as desired*(1-0.1)
            targetMet = (currentRPM >= targetRPM);

            // Note: Telemetry must show currentRPM, targetRPM, targetMet on separate lines
            telemetry.clear();
            telemetry.addData("currentRPM", String.format("%.1f", currentRPM));
            telemetry.addData("targetRPM", String.format("%.1f", targetRPM));
            telemetry.addData("current speed", shooter.getVelocity());
            telemetry.addData("targetMet", targetMet);
            telemetry.update();

            // --- Pusher servo (gamepad2 RB) ---
            // If RB pressed, rotate clockwise 80°, then return counterclockwise 80° and set power to 0
            if (gamepad2.right_bumper) {
                // rotate to +80 degrees then back to 0
                runServoPushCycle(pusher, 80.0);
            }

            // --- Carousel control (gamepad2) ---
            // Button combos:
            // LB + dpad_right => rotate current position +60°
            // LB + dpad_left  => rotate current position -60°
            // dpad_right => +120°
            // dpad_left  => -120°
            if (gamepad2.dpad_right && gamepad2.left_bumper) {
                rotateCarouselByDegrees(60.0);
                sleep(200); // simple debounce
            } else if (gamepad2.dpad_left && gamepad2.left_bumper) {
                rotateCarouselByDegrees(-60.0);
                sleep(200);
            } else if (gamepad2.dpad_right) {
                rotateCarouselByDegrees(120.0);
                sleep(200);
            } else if (gamepad2.dpad_left) {
                rotateCarouselByDegrees(-120.0);
                sleep(200);
            }

            // Short sleep to avoid busy loop and allow controller updates
            idle();
        }

        // stop all motors when done
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        intake.setPower(0);
        shooter.setPower(0);
        carousel.setPower(0);
    }

    // Helper: set servo to an angle in degrees where 0 is neutral and range spans SERVO_RANGE_DEGREES
    private void setServoAngleDegrees(Servo servo, double degrees) {
        // Map degrees in [-150, +150] (or 0..300 if you prefer) into 0..1
        double normalized = (degrees + (SERVO_RANGE_DEGREES / 2.0)) * SERVO_TO_POSITION_FACTOR;
        normalized = Range.clip(normalized, 0.0, 1.0);
        servo.setPosition(normalized);
    }

    // Helper to perform the push cycle: rotate +angle then back to original 0
    private void runServoPushCycle(Servo servo, double angleDegrees) {
        // move clockwise (positive angle) then return
        setServoAngleDegrees(servo, angleDegrees);
        // Wait for servo to reach (time needed depends on servo speed; small sleep)
        sleep(250);
        // return to 0
        setServoAngleDegrees(servo, 0.0);
        sleep(250);
        // stop power is implicit for standard hobby servos; no further action required
    }

    // Helper to set shooter motor to a target RPM value
    // We set power proportionally. For a simple approach, map RPM target to motor power with clamp.
    // Then we compute the "targetRPM" variable as desired*(1-0.1) as specified.
    private void setShooterTargetRPM(double desiredRPM) {
        // Simple proportional mapping from target RPM to motor power (tune as needed)
        // We'll map 6000 RPM -> power 1.0 (GoBilda spec), but clamp to [-1,1]
        double power = desiredRPM / 6000.0;
        power = Range.clip(power, -1.0, 1.0);
        shooter.setPower(power);

        // Update targetRPM to desired*(1-0.1)
        targetRPM = desiredRPM * (1.0 - 0.1);
        // Immediately read currentRPM after commanding; actual RPM read occurs in loop
        // We'll update targetMet after reading actual currentRPM in main loop
    }

    // Helper to rotate the carousel by a specified deltaDegrees relative to current position.
    // Keeps the internal angle in [0,360).
    private void rotateCarouselByDegrees(double deltaDegrees) {
        // Read current encoder ticks -> current output shaft degrees
        int currentTicks = carousel.getCurrentPosition();
        // Convert ticks to output-shaft degrees
        double currentOutputRevs = currentTicks / CAROUSEL_COUNTS_PER_OUTPUT_REV;
        double currentAngle = (currentOutputRevs * 360.0) % 360.0;
        if (currentAngle < 0) currentAngle += 360.0;

        double newAngle = (currentAngle + deltaDegrees) % 360.0;
        if (newAngle < 0) newAngle += 360.0;

        // Compute target ticks for newAngle
        double targetOutputRevs = newAngle / 360.0;
        int targetTicks = (int) Math.round(targetOutputRevs * CAROUSEL_COUNTS_PER_OUTPUT_REV);

        // Move using RUN_TO_POSITION
        carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        carousel.setTargetPosition(targetTicks);

        // Set a reasonable power for movement (tune as needed)
        carousel.setPower(0.5);
        // Wait until reach or timeout
        long start = System.currentTimeMillis();
        long timeoutMs = 2000;
        while (opModeIsActive() && carousel.isBusy() && System.currentTimeMillis() - start < timeoutMs) {
            idle();
        }
        // stop and return to RUN_USING_ENCODER
        carousel.setPower(0.0);
        carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
