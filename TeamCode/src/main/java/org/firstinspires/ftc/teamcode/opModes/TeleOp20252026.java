package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOp20252026", group = "Competition")
public class TeleOp20252026 extends LinearOpMode {

    // Drive motors (Chassis)
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    // Expansion Hub motors
    private DcMotorEx carousel;    // needs encoder + RUN_TO_POSITION
    private DcMotor intake;
    private DcMotorEx shooter;     // shooter with encoder/velocity feedback

    // Servo
    private Servo pusher;

    // Shooter RPM tracking
    private double currentRPM = 0.0;
    private double targetRPM = 0.0;
    private boolean atTargetRPM = false;

    // Constants and hardware assumptions (adjust to your hardware)
    private static final int COUNTS_PER_REV = 28;             // encoder CPR (example for many motors)
    private static final double CAROUSEL_GEAR_RATIO = 99.5;  // from gearbox reference (60 rpm gearmotor)
    private static final double SHOOTER_GEAR_RATIO = 1.0;    // if gearbox present change accordingly
    private static final double TICKS_PER_DEGREE_CAROUSEL = (COUNTS_PER_REV * CAROUSEL_GEAR_RATIO) / 360.0;
    private static final double TICKS_PER_REV_SHOOTER = COUNTS_PER_REV * SHOOTER_GEAR_RATIO;

    // Servo mapping for multimode smart servo in Standard (±150°) -> 300° range
    // Servo position 0.5 maps to 0 degrees. Position = 0.5 + degrees/300
    private static final double SERVO_CENTER = 0.5;
    private static final double SERVO_MOVE_DEGREES = 80.0;
    private static final double SERVO_MOVE_POSITION = SERVO_CENTER + (SERVO_MOVE_DEGREES / 300.0);

    // Drive speed limits
    private static final double MAX_DRIVE_POWER = 1.0;
    private static final double LOW_SPEED_LIMIT = 0.30;

    @Override
    public void runOpMode() throws InterruptedException {

        // Hardware map
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        carousel = hardwareMap.get(DcMotorEx.class, "carousel");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        pusher = hardwareMap.get(Servo.class, "pusher");

        // Motor directions - adjust if your robot moves reversed
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // Shooter and carousel direction / behavior
        carousel.setDirection(DcMotorSimple.Direction.FORWARD);
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Reset encoders for carousel and shooter
        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        carousel.setTargetPosition(0);
        carousel.setPower(0.0);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize pusher servo to 0 degrees (center mapping)
        pusher.setPosition(SERVO_CENTER);

        // Telemetry initial state
        telemetry.addData("Init", "Complete");
        telemetry.update();

        waitForStart();

        // Keep track of carousel position in encoder ticks
        int carouselTargetTicks = 0;

        while (opModeIsActive()) {

            // -------------------
            // Driving (gamepad1)
            // -------------------
            // Left joystick: translation (forward/back/strafe)
            double driveY = -gamepad1.left_stick_y;   // forward is negative on stick
            double driveX = gamepad1.left_stick_x;    // strafe
            // Right joystick X: rotation
            double rot = gamepad1.right_stick_x;

            // Low speed mode when RT touched/held (gamepad1.right_trigger)
            boolean lowSpeed = gamepad1.right_trigger > 0.05;

            double speedLimit = lowSpeed ? LOW_SPEED_LIMIT : MAX_DRIVE_POWER;

            // Combine for mecanum drive
            // Basic mecanum formula: FL = y + x + r ; FR = y - x - r ; BL = y - x + r ; BR = y + x - r
            double fl = driveY + driveX + rot;
            double fr = driveY - driveX - rot;
            double bl = driveY - driveX + rot;
            double br = driveY + driveX - rot;

            // Scale by joystick magnitude and speedLimit
            double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)));
            if (max > 1.0) {
                fl /= max;
                fr /= max;
                bl /= max;
                br /= max;
            }

            fl = Range.clip(fl * speedLimit, -1.0, 1.0);
            fr = Range.clip(fr * speedLimit, -1.0, 1.0);
            bl = Range.clip(bl * speedLimit, -1.0, 1.0);
            br = Range.clip(br * speedLimit, -1.0, 1.0);

            leftFront.setPower(fl);
            rightFront.setPower(fr);
            leftBack.setPower(bl);
            rightBack.setPower(br);

            // -------------------
            // Intake control (gamepad1)
            // -------------------
            // LT on gamepad1: intake backwards full power
            boolean intakeBack = gamepad1.left_trigger > 0.05;
            // LB on gamepad1: intake forward; intensity controlled by left_trigger value (assumption)
            boolean intakeForwardPressed = gamepad1.left_bumper;
            double intakePower = 0.0;

            // A on gamepad1 held: limit intake to 30% absolute power
            boolean intakeLimit = gamepad1.a;

            if (intakeBack) {
                intakePower = -1.0; // backwards full power
            } else if (intakeForwardPressed) {
                // Use left_trigger as intensity when LB held (assumption to allow variable intensity)
                // If left_trigger is used for backward, we use right_trigger for intensity or preserve that left_trigger value is used here
                double intensity = gamepad1.left_trigger; // assumption: left_trigger pressure used for intensity when LB held
                if (intensity < 0.05) {
                    intensity = 1.0; // if no pressure, default to full speed forward when LB held
                }
                intakePower = Range.clip(intensity, 0.0, 1.0);
            } else {
                intakePower = 0.0;
            }

            if (intakeLimit) {
                intakePower = Range.clip(intakePower, -LOW_SPEED_LIMIT, LOW_SPEED_LIMIT);
            }

            intake.setPower(intakePower);

            // -------------------
            // Shooter control (gamepad2)
            // -------------------
            // Use DcMotorEx.getVelocity() if available to compute RPM.
            // Some SDKs report ticks per second; formula below converts encoder ticks/sec to RPM.
            currentRPM = 0.0;
            try {
                // getVelocity returns ticks per second for many DcMotorEx implementations
                double ticksPerSecond = shooter.getVelocity(); // units depend on implementation
                // If getVelocity returns ticks per second:
                // RPM = (ticksPerSecond / ticksPerRev) * 60
                currentRPM = (ticksPerSecond / TICKS_PER_REV_SHOOTER) * 60.0;
            } catch (Exception e) {
                // if getVelocity not supported, leave currentRPM unchanged or estimate from encoder deltas (not implemented)
                currentRPM = 0.0;
            }

            // Buttons on gamepad2 to set RPM
            if (gamepad2.x) {
                // set motor to reach 1000 RPM setpoint; targetRPM variable to 10% less than set RPM
                setShooterTargetRPM(1000.0);
            }
            if (gamepad2.a) {
                setShooterTargetRPM(2500.0);
            }
            if (gamepad2.b) {
                setShooterTargetRPM(5000.0);
            }
            if (gamepad2.y) {
                // stop shooter
                targetRPM = 0.0;
                setShooterPowerFromRPM(0.0);
            }

            // Simple power control: map desired RPM to a power value (open-loop)
            // Here we assume max RPM corresponds to 1.0 power (adjust with empirical tuning)
            // When targetRPM is zero we already set power to 0.
            if (targetRPM > 0.0) {
                setShooterPowerFromRPM(targetRPM / 0.9); // set nominal motor RPM slightly above target, since targetRPM stored = 0.9*set
            }

            // Update atTargetRPM boolean: within 5% tolerance considered "met"
            if (targetRPM <= 0.0) {
                atTargetRPM = targetRPM == 0.0 && Math.abs(currentRPM) < 10.0;
            } else {
                atTargetRPM = Math.abs(currentRPM - targetRPM) <= (0.05 * targetRPM);
            }

            // -------------------
            // Servo (pusher) control (gamepad2)
            // -------------------
            if (gamepad2.right_bumper) {
                // Rotate clockwise 80 degrees (move up), then immediately back
                pusher.setPosition(SERVO_MOVE_POSITION);
                // small sleep to allow servo to move
                sleep(180); // milliseconds; tune as needed
                pusher.setPosition(SERVO_CENTER);
                // debounce to avoid repeated triggers in a single press
                while (gamepad2.right_bumper && opModeIsActive()) {
                    idle();
                }
            }

            // -------------------
            // Carousel control (gamepad2)
            // -------------------
            // Initialize already done; carouselTargetTicks keeps current target
            // Commands:
            // LB + D-Pad R: rotate 60 degrees right (relative)
            // LB + D-Pad L: rotate 60 degrees left (relative)
            // D-Pad R: rotate 120 degrees right
            // D-Pad L: rotate 120 degrees left
            boolean gp2LB = gamepad2.left_bumper;
            boolean dpadRight = gamepad2.dpad_right;
            boolean dpadLeft = gamepad2.dpad_left;

            if (gp2LB && dpadRight) {
                carouselTargetTicks += (int) Math.round(60.0 * TICKS_PER_DEGREE_CAROUSEL);
                carousel.setTargetPosition(carouselTargetTicks);
                carousel.setPower(0.5);
                waitUntilCarouselAtTarget();
                carousel.setPower(0.0);
                // debounce
                while (gamepad2.left_bumper && gamepad2.dpad_right && opModeIsActive()) { idle(); }
            } else if (gp2LB && dpadLeft) {
                carouselTargetTicks -= (int) Math.round(60.0 * TICKS_PER_DEGREE_CAROUSEL);
                carousel.setTargetPosition(carouselTargetTicks);
                carousel.setPower(0.5);
                waitUntilCarouselAtTarget();
                carousel.setPower(0.0);
                while (gamepad2.left_bumper && gamepad2.dpad_left && opModeIsActive()) { idle(); }
            } else if (dpadRight) {
                carouselTargetTicks += (int) Math.round(120.0 * TICKS_PER_DEGREE_CAROUSEL);
                carousel.setTargetPosition(carouselTargetTicks);
                carousel.setPower(0.5);
                waitUntilCarouselAtTarget();
                carousel.setPower(0.0);
                while (gamepad2.dpad_right && opModeIsActive()) { idle(); }
            } else if (dpadLeft) {
                carouselTargetTicks -= (int) Math.round(120.0 * TICKS_PER_DEGREE_CAROUSEL);
                carousel.setTargetPosition(carouselTargetTicks);
                carousel.setPower(0.5);
                waitUntilCarouselAtTarget();
                carousel.setPower(0.0);
                while (gamepad2.dpad_left && opModeIsActive()) { idle(); }
            }

            // -------------------
            // Telemetry (three lines)
            // -------------------
            telemetry.clearAll();
            telemetry.addData("Shooter RPM (actual)", "%.1f", currentRPM);
            telemetry.addData("Target RPM", "%.1f", targetRPM);
            telemetry.addData("Target Met", "%s", atTargetRPM ? "True" : "False");
            telemetry.update();

            idle();
        }
    }

    // Set the targetRPM variable to 90% of the requested setpoint
    private void setShooterTargetRPM(double setpointRPM) {
        targetRPM = setpointRPM * 0.90; // per spec: target variable = 10% less than set RPM
    }

    // Map an intended motor RPM to an open-loop power and apply to shooter motor
    // This is a simple proportional mapping and will require tuning on actual robot
    private void setShooterPowerFromRPM(double requestedRPM) {
        // Assumption: maximum motor free RPM corresponds to some known RPM (e.g., 6000 for that motor)
        double motorFreeRPM = 6000.0; // for GoBilda 5202 6000 rpm nominal; change if gearbox or different spec
        double power = Range.clip(requestedRPM / motorFreeRPM, 0.0, 1.0);
        shooter.setPower(power);
    }

    // Wait until carousel reaches its target position (simple blocking wait)
    // Note: this blocks loop while moving; it's acceptable for single-step carousel movements
    private void waitUntilCarouselAtTarget() {
        while (opModeIsActive() && carousel.isBusy()) {
            idle();
        }
    }
}
