package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOp20252026", group = "TeleOp")
public class TeleOp20252026 extends OpMode {

    // Drive motors (Control Hub)
    private DcMotorEx leftFront, rightFront, leftBack, rightBack;

    // Expansion Hub motors
    private DcMotorEx carousel;      // GoBilda 5202 60 rpm planetary (with gearbox)
    private DcMotorEx intake;        // Tetrix
    private DcMotorEx shooter;       // GoBilda 5202 6000 rpm

    // Servo (Control Hub multimode smart servo)
    private Servo pusherServo;

    // Shooter RPM tracking variables
    private double currentRPM = 0.0;
    private double targetRPM = 0.0;
    private boolean targetMet = false;

    // Carousel encoder conversion (assumes encoder CPR and gearbox)
    // If encoder CPR differs, change this value accordingly.
    private static final double ENCODER_CPR = 28.0;            // common CPR for many DC encoders; change if different
    private static final double CAROUSEL_GEAR_RATIO = 99.5;    // gearbox ratio for 60 RPM variant
    private static final double CAROUSEL_TICKS_PER_OUTPUT_REV = ENCODER_CPR * CAROUSEL_GEAR_RATIO; // ≈ 2786

    // Servo angle mapping (Studica multimode smart servo standard mode ±150° maps to 0..1)
    private static final double SERVO_MIN_DEG = -150.0;
    private static final double SERVO_MAX_DEG = 150.0;

    // Pusher servo target angle when pressed (clockwise 80 degrees)
    private static final double PUSHER_SWING_DEG = 80.0;

    // Internal carousel tracking angle in [0, 360)
    private double carouselAngle = 0.0;

    // Helper: map degrees in [-150,150] to servo position [0,1]
    private double servoPositionFromDegrees(double degrees) {
        double clamped = Math.max(SERVO_MIN_DEG, Math.min(SERVO_MAX_DEG, degrees));
        return (clamped - SERVO_MIN_DEG) / (SERVO_MAX_DEG - SERVO_MIN_DEG);
    }

    @Override
    public void init() {
        // Drive motors
        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotorEx.class, "rightBack");

        // Set directions: adjust if your robot's wiring requires different signs
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // Expansion hub motors
        carousel = hardwareMap.get(DcMotorEx.class, "Carousel");
        intake   = hardwareMap.get(DcMotorEx.class, "Intake");
        shooter  = hardwareMap.get(DcMotorEx.class, "Shooter");

        // Set directions for expansion motors as needed
        carousel.setDirection(DcMotorSimple.Direction.FORWARD); // logical forward
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        // Use encoders where appropriate
        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // simple power control
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Servo
        pusherServo = hardwareMap.get(Servo.class, "pusher");

        // Initialize servo to 0 degrees (mapped to position)
        double startServoAngle = 0.0;
        pusherServo.setPosition(servoPositionFromDegrees(startServoAngle));

        // Initialize carousel angle and hold position (motor stopped)
        carouselAngle = 0.0;
        carousel.setPower(0.0);

        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ------------------
        // Driving (gamepad1 left stick for translation, right stick x for rotation)
        // - Left joystick: y = forward/back, x = strafe
        // - Right joystick x: rotation (spin on spot)
        double driveY = -gamepad1.left_stick_y; // forward is negative on stick
        double driveX = gamepad1.left_stick_x;  // strafing
        double turn   = gamepad1.right_stick_x; // rotation

        // Speed scaling: RT on gamepad1 low-speed (RT ranges 0..1)
        boolean lowSpeed = gamepad1.right_trigger > 0.05;
        double maxPower = lowSpeed ? 0.30 : 1.0;

        // Scale joystick values to control speed smoothly
        double driveScale = Math.hypot(driveX, driveY);
        if (driveScale > 1.0) driveScale = 1.0;

        // Mecanum mixing
        double lf = driveY + driveX + turn;
        double rf = driveY - driveX - turn;
        double lb = driveY - driveX + turn;
        double rb = driveY + driveX - turn;

        // Normalize
        double max = Math.max(Math.max(Math.abs(lf), Math.abs(rf)),
                Math.max(Math.abs(lb), Math.abs(rb)));
        if (max > 1.0) {
            lf /= max;
            rf /= max;
            lb /= max;
            rb /= max;
        }

        // Apply maxPower multiplier
        leftFront.setPower(Range.clip(lf * maxPower, -1.0, 1.0));
        rightFront.setPower(Range.clip(rf * maxPower, -1.0, 1.0));
        leftBack.setPower(Range.clip(lb * maxPower, -1.0, 1.0));
        rightBack.setPower(Range.clip(rb * maxPower, -1.0, 1.0));

        // ------------------
        // Intake Motor (gamepad1)
        // LB -> backwards full power
        // LT -> forward proportional to trigger value
        // A -> limits intake max to 30% when held (both directions)
        double intakePower = 0.0;
        final double intakeLimitIfA = gamepad1.a ? 0.30 : 1.0;

        if (gamepad1.left_bumper) {
            // backwards full power (negative forward)
            intakePower = -1.0 * intakeLimitIfA;
        } else if (gamepad1.left_trigger > 0.05) {
            intakePower = Range.clip(gamepad1.left_trigger * intakeLimitIfA, -1.0, 1.0);
        } else {
            intakePower = 0.0;
        }
        intake.setPower(intakePower);

        // ------------------
        // Shooter Motor (gamepad2)
        // X -> target 1000 RPM
        // A -> target 2000 RPM
        // B -> target 5000 RPM
        // Y -> target 0 RPM
        // Use actual RPM to set currentRPM and check targetMet.
        // We will read shooter.getVelocity() and convert to RPM assuming SDK returns ticks/sec.
        // If getVelocity() already returns RPM, conversion below will need to be simplified.

        // Here we assume encoder CPR is ENCODER_CPR ticks per motor rev.
        // shooter.getVelocity() returns ticks/sec in many SDKs; convert ticks/sec -> RPM:
        // RPM = (ticksPerSecond / ticksPerRevolution) * 60
        double shooterTicksPerRev = ENCODER_CPR; // adjust if different for your encoder
        double shooterTicksPerSecond = shooter.getVelocity(); // SDK may return ticks/sec; test and adjust
        currentRPM = (shooterTicksPerSecond / shooterTicksPerRev) * 60.0;

        // Buttons set "desired" RPM and then we set targetRPM internally as 90% of desired
        if (gamepad2.x) {
            double desired = 1000.0;
            targetRPM = desired * (1.0 - 0.1);
            // Spin up: simple proportional power to reach desired; tune as needed
            double power = pidPowerForRPM(currentRPM, desired);
            shooter.setPower(power);
        } else if (gamepad2.a) {
            double desired = 2000.0;
            targetRPM = desired * (1.0 - 0.1);
            double power = pidPowerForRPM(currentRPM, desired);
            shooter.setPower(power);
        } else if (gamepad2.b) {
            double desired = 5000.0;
            targetRPM = desired * (1.0 - 0.1);
            double power = pidPowerForRPM(currentRPM, desired);
            shooter.setPower(power);
        } else if (gamepad2.y) {
            double desired = 0.0;
            targetRPM = desired * (1.0 - 0.1);
            shooter.setPower(0.0);
        } // if no button pressed we leave shooter power as previously set (or 0 at init)

        // Update targetMet boolean
        targetMet = currentRPM >= targetRPM;

        // ------------------
        // Pusher Servo (gamepad2 RB)
        // If RB pressed: rotate clockwise 80 degrees then back immediately.
        if (gamepad2.right_bumper) {
            // Current is assumed 0 degrees at start; move to +80 then back to 0
            double startAngle = 0.0;
            double forwardAngle = startAngle + PUSHER_SWING_DEG;
            pusherServo.setPosition(servoPositionFromDegrees(forwardAngle));
            // Small delay simulation: in iterative OpMode, use telemetry or state machine to step back next loop
            // For simplicity, we immediately set back to start position next line
            pusherServo.setPosition(servoPositionFromDegrees(startAngle));
        }

        // ------------------
        // Carousel control (gamepad2)
        // Modes:
        // LB + dpad right -> +60 degrees
        // LB + dpad left  -> -60 degrees
        // dpad right -> +120 degrees
        // dpad left -> -120 degrees
        // Else: keep motor stopped at frozen position
        // We will implement a simple state that starts a RUN_TO_POSITION move, then stops and updates carouselAngle.
        if (carouselBusy()) {
            // allowing RUN_TO_POSITION to finish
        } else {
            boolean commandProcessed = false;
            if (gamepad2.left_bumper && gamepad2.dpad_right) {
                rotateCarouselByDegrees(60.0);
                commandProcessed = true;
            } else if (gamepad2.left_bumper && gamepad2.dpad_left) {
                rotateCarouselByDegrees(-60.0);
                commandProcessed = true;
            } else if (gamepad2.dpad_right) {
                rotateCarouselByDegrees(120.0);
                commandProcessed = true;
            } else if (gamepad2.dpad_left) {
                rotateCarouselByDegrees(-120.0);
                commandProcessed = true;
            }

            if (!commandProcessed) {
                // Hold position (brake)
                carousel.setPower(0.0);
                carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }

        // ------------------
        // Telemetry (three lines only, dynamic)
        telemetry.clear();
        telemetry.addData("currentRPM", "%.1f", currentRPM);
        telemetry.addData("targetRPM", "%.1f", targetRPM);
        telemetry.addData("targetMet", targetMet);
        telemetry.update();
    }

    // Simple helper method to check if carousel is busy RUN_TO_POSITION
    private boolean carouselBusy() {
        return (carousel.getMode() == DcMotor.RunMode.RUN_TO_POSITION) && carousel.isBusy();
    }

    // Rotate carousel by deltaDegrees relative to current carouselAngle; stops motor on target
    private void rotateCarouselByDegrees(double deltaDegrees) {
        // Compute new angle in [0,360)
        double newAngle = (carouselAngle + deltaDegrees) % 360.0;
        if (newAngle < 0) newAngle += 360.0;
        carouselAngle = newAngle;

        // Compute target ticks for that angle
        double targetTicks = (carouselAngle / 360.0) * CAROUSEL_TICKS_PER_OUTPUT_REV;
        int targetPosition = (int)Math.round(targetTicks);

        // Configure motor for RUN_TO_POSITION
        carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // ensure encoder mode
        carousel.setTargetPosition(targetPosition);
        carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Give a moderate power; motor will stop at target and hold (we then set power to 0)
        carousel.setPower(0.5);
    }

    // Simple proportional power for shooter based on RPM error
    // This is a crude P-controller tuned moderately; you can tune Kp as required
    private double pidPowerForRPM(double current, double desired) {
        double error = desired - current;
        double Kp = 0.0005; // very small Kp because RPMs are large; tune as necessary
        double power = Kp * error;
        power = Range.clip(power, -1.0, 1.0);
        // If desired is zero, kill power
        if (desired <= 0.0) power = 0.0;
        return power;
    }

    @Override
    public void stop() {
        // Stop all motors
        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftBack.setPower(0.0);
        rightBack.setPower(0.0);
        intake.setPower(0.0);
        shooter.setPower(0.0);
        carousel.setPower(0.0);
        // Reset servo to 0 degrees
        pusherServo.setPosition(servoPositionFromDegrees(0.0));
    }
}
