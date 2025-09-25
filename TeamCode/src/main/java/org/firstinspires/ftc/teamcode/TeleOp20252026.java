package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOp20252026", group = "TeleOp")
public class TeleOp20252026 extends OpMode {

    // Drive motors (4): leftFront, leftBack, rightFront, rightBack
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;

    // Intake motor
    private DcMotor intake;

    // Shooter motors (2 identical)
    private DcMotorEx shooterLeft, shooterRight;

    // Servos: continuous-mode smart servo (carousel), normal-mode smart servo (shooter pusher)
    private Servo carouselServo;       // continuous rotation servo (we model as position 0-1 mapped to 0-360)
    private Servo shooterServo;        // normal servo (variable declared, left blank behavior)

    // Runtime / helpers
    private final ElapsedTime runtime = new ElapsedTime();

    // Intake toggle state
    private boolean intakeOnFull = false;
    private boolean prevGamepad1Lb = false;

    // Low speed mode variable
    private boolean lowSpeedMode = false;

    // Shooter RPM control
    private double targetRpm = 0.0;
    private final double RPM_TOLERANCE = 50.0; // RPM tolerance for "at target" check
    private boolean bothShootersAtTarget = false;

    // Carousel continuous servo angle tracking (degrees). Start at 0 degrees.
    private double carouselAngleDeg = 0.0;

    // Shooter servo (normal) shot state
    private boolean shooterServoActive = false;
    private final double SHOOTER_ROTATE_DEG = 80.0; // up about 80 degrees (clockwise)
    private double shooterServoRestPos = 0.5; // placeholder neutral pos 0..1 (user to tune)
    private double shooterServoUpPos = shooterServoRestPos + (SHOOTER_ROTATE_DEG / 300.0); // map deg to 0..1 (approx)
    private final ElapsedTime shooterServoTimer = new ElapsedTime();
    private final double SHOOTER_UP_DURATION = 0.25; // seconds hold up before returning

    // Encoder to RPM conversion (encoder ticks per motor revolution)
    // Set to encoder ticks per revolution of the motor's output shaft.
    // If your encoder/report returns ticks per revolution different from this, update it.
    private final double ENCODER_TICKS_PER_REV = 28.0; // typical for Yellow Jacket example (adjust if different)

    // Conversion helper: velocity from DcMotorEx.getVelocity() returns ticks per second for many SDKs.
    // RPM = ticksPerSecond * 60 / ticksPerRev
    // To set velocity we compute ticksPerSecond = targetRpm * ticksPerRev / 60

    @Override
    public void init() {
        // Map hardware (update names as needed)
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack  = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack  = hardwareMap.get(DcMotorEx.class, "rightBack");

        intake = hardwareMap.get(DcMotor.class, "intake");

        shooterLeft  = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");

        carouselServo = hardwareMap.get(Servo.class, "carouselServo");
        shooterServo  = hardwareMap.get(Servo.class, "shooterServo");

        // Motor directions - adjust if actual wiring produces reversed motion
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        shooterLeft.setDirection(DcMotor.Direction.FORWARD);
        shooterRight.setDirection(DcMotor.Direction.REVERSE);

        // Ensure motors stop
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        intake.setPower(0);
        shooterLeft.setPower(0);
        shooterRight.setPower(0);

        // Initialize servos
        // Continuous servo starts at 0 degrees -> map to position 0.0
        carouselAngleDeg = 0.0;
        carouselServo.setPosition(angleToServoPosition(carouselAngleDeg)); // set to 0

        // shooterServo left blank but define neutral; operator may calibrate shooterServoRestPos before match
        shooterServo.setPosition(shooterServoRestPos);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();
        shooterServoTimer.reset();
    }

    @Override
    public void loop() {
        // -------------------------
        // DRIVER (gamepad1) controls
        // -------------------------
        // Low speed mode - if RT touched or held -> limit to 30%
        lowSpeedMode = gamepad1.right_trigger > 0.05;

        // Driving: left joystick controls translation (strafe + forward/back)
        double leftX = gamepad1.left_stick_x;    // strafe: left negative, right positive
        double leftY = -gamepad1.left_stick_y;   // forward positive (invert joystick)
        // Right joystick controls rotation
        double rotate = gamepad1.right_stick_x;  // left negative -> CCW, right positive -> CW

        // Compute driver powers (simple mecanum-like mixing for holonomic strafing)
        // Basic arcade combination: forward/back + strafe + rotate
        double rf = leftY - leftX + rotate; // rightFront
        double rb = leftY + leftX + rotate; // rightBack
        double lf = leftY + leftX - rotate; // leftFront
        double lb = leftY - leftX - rotate; // leftBack

        // Scale so max magnitude is <= 1
        double max = Math.max(
                Math.max(Math.abs(lf), Math.abs(lb)),
                Math.max(Math.abs(rf), Math.abs(rb))
        );
        if (max > 1.0) {
            lf /= max;
            lb /= max;
            rf /= max;
            rb /= max;
        }

        // Apply low speed scaling if engaged
        double speedScale = lowSpeedMode ? 0.30 : 1.0;
        lf *= speedScale;
        lb *= speedScale;
        rf *= speedScale;
        rb *= speedScale;

        // Set drive powers
        leftFront.setPower(clamp(lf));
        leftBack.setPower(clamp(lb));
        rightFront.setPower(clamp(rf));
        rightBack.setPower(clamp(rb));

        // Intake: gamepad1 LB toggle (press once to start full intake, press again to go to slow resting)
        if (gamepad1.left_bumper && !prevGamepad1Lb) {
            intakeOnFull = !intakeOnFull;
        }
        prevGamepad1Lb = gamepad1.left_bumper;

        // Intake power modes:
        // - intakeOnFull true => full forward (1.0)
        // - if intakeOnFull false and not reversed by LT => stopped/slight holding speed (0.15)
        // - LT (left trigger on driver) reverses intake proportionally (more pull => faster reverse)
        double intakePower = 0.0;
        double driverLt = gamepad1.left_trigger;
        if (driverLt > 0.05) {
            // reverse intake proportional to amount pulled
            intakePower = -driverLt; // negative = reverse
        } else {
            if (intakeOnFull) {
                intakePower = 1.0;
            } else {
                intakePower = 0.15; // slow resting state
            }
        }
        intake.setPower(clamp(intakePower));

        // -------------------------
        // OPERATOR (gamepad2) controls
        // -------------------------
        // Shooter RPM presets
        if (gamepad2.x) {
            targetRpm = 1000.0;
            setShooterTargetRpm(targetRpm);
        } else if (gamepad2.a) {
            targetRpm = 2500.0;
            setShooterTargetRpm(targetRpm);
        } else if (gamepad2.b) {
            targetRpm = 5000.0;
            setShooterTargetRpm(targetRpm);
        } else if (gamepad2.y) {
            // stop shooter motors
            targetRpm = 0.0;
            shooterLeft.setPower(0.0);
            shooterRight.setPower(0.0);
        }

        // Update shooter at-target boolean based on measured RPMs
        double rpmLeft = getMotorRpm(shooterLeft);
        double rpmRight = getMotorRpm(shooterRight);
        bothShootersAtTarget = (Math.abs(rpmLeft - targetRpm) <= RPM_TOLERANCE)
                && (Math.abs(rpmRight - targetRpm) <= RPM_TOLERANCE)
                && (targetRpm > 0.0);

        // Operator RT = shoot ball: rotate normal servo up then back down ~80 degrees
        if (gamepad2.right_trigger > 0.05 && !shooterServoActive) {
            // start shoot sequence
            shooterServoActive = true;
            shooterServoTimer.reset();
            // rotate up (clockwise)
            shooterServo.setPosition(clampServo(shooterServoUpPos));
        }

        if (shooterServoActive) {
            if (shooterServoTimer.seconds() >= SHOOTER_UP_DURATION) {
                // return to rest (counterclockwise)
                shooterServo.setPosition(clampServo(shooterServoRestPos));
                shooterServoActive = false;
            }
        }

        // Carousel continuous servo controls (gamepad2)
        // LB + DPad R -> rotate 60 degrees right (add +60)
        // LB + DPad L -> rotate 60 degrees left (add -60)
        // DPad R -> rotate 120 degrees right (+120)
        // DPad L -> rotate 120 degrees left (-120)
        // We modify carouselAngleDeg and set servo position accordingly.
        if (gamepad2.left_bumper && gamepad2.dpad_right) {
            carouselAngleDeg += 60.0;
            normalizeCarouselAngle();
            carouselServo.setPosition(angleToServoPosition(carouselAngleDeg));
        } else if (gamepad2.left_bumper && gamepad2.dpad_left) {
            carouselAngleDeg -= 60.0;
            normalizeCarouselAngle();
            carouselServo.setPosition(angleToServoPosition(carouselAngleDeg));
        } else if (gamepad2.dpad_right) {
            carouselAngleDeg += 120.0;
            normalizeCarouselAngle();
            carouselServo.setPosition(angleToServoPosition(carouselAngleDeg));
        } else if (gamepad2.dpad_left) {
            carouselAngleDeg -= 120.0;
            normalizeCarouselAngle();
            carouselServo.setPosition(angleToServoPosition(carouselAngleDeg));
        }

        // Telemetry - show shooter RPMs and whether both are at desired RPM
        telemetry.addData("Target RPM", "%.0f", targetRpm);
        telemetry.addData("Shooter RPM Left", "%.1f", rpmLeft);
        telemetry.addData("Shooter RPM Right", "%.1f", rpmRight);
        telemetry.addData("BothAtTarget", bothShootersAtTarget);
        telemetry.addData("LowSpeedMode", lowSpeedMode);
        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop everything
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        intake.setPower(0);
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
    }

    // -------------------------
    // Helper methods
    // -------------------------
    private void setShooterTargetRpm(double rpm) {
        if (rpm <= 0.0) {
            shooterLeft.setPower(0.0);
            shooterRight.setPower(0.0);
            return;
        }

        // Convert rpm to ticks per second for setVelocity
        double ticksPerSecond = rpm * ENCODER_TICKS_PER_REV / 60.0;
        // The SDK's setVelocity expects ticks per second for DcMotorEx in many builds;
        // if your SDK expects different units, adapt accordingly.
        shooterLeft.setVelocity(ticksPerSecond);
        shooterRight.setVelocity(ticksPerSecond);
    }

    private double getMotorRpm(DcMotorEx motor) {
        // getVelocity() returns encoder ticks per second for many versions of the SDK.
        // Convert to RPM.
        double ticksPerSecond = motor.getVelocity(); // check your SDK: this may be ticks/sec
        return ticksPerSecond * 60.0 / ENCODER_TICKS_PER_REV;
    }

    private double clamp(double v) {
        if (v > 1.0) return 1.0;
        if (v < -1.0) return -1.0;
        return v;
    }

    private double clampServo(double pos) {
        if (pos < 0.0) return 0.0;
        if (pos > 1.0) return 1.0;
        return pos;
    }

    private double angleToServoPosition(double angleDeg) {
        // Map angle 0..360 to servo position 0..1
        double a = angleDeg % 360.0;
        if (a < 0) a += 360.0;
        return clampServo(a / 360.0);
    }

    private void normalizeCarouselAngle() {
        // keep angle in -180..+180 range for easier reasoning (optional)
        while (carouselAngleDeg > 180.0) carouselAngleDeg -= 360.0;
        while (carouselAngleDeg <= -180.0) carouselAngleDeg += 360.0;
    }
}
