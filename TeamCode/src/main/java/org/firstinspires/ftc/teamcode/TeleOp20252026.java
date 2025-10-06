package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOp20252026", group = "TeleOp")
public class TeleOp20252026 extends LinearOpMode {

    // Drive motors
    private DcMotor leftFront, leftBack, rightFront, rightBack;

    // Other motors
    private DcMotor intake;
    private DcMotorEx shooterLeft, shooterRight; // use DcMotorEx for velocity control
    private DcMotorEx carousel; // encoder based position

    // Servo
    private Servo shooterServo; // standard-mode servo mapped to 0..1

    // Constants (tune these)
    private static final double MAX_DRIVE_POWER = 1.0;
    private static final double LOW_SPEED_FACTOR = 0.30; // 30% when RT pressed by driver
    // Encoder ticks per motor revolution (assumption: 28 CPR internal encoder). Adjust to your motor encoder CPR.
    private static final double MOTOR_TICKS_PER_REV = 28.0;

    // Shooter motor gear ratio: 1:1 (per your reference). If not, adjust.
    private static final double SHOOTER_GEAR_RATIO = 1.0;

    // Carousel gearbox ratio (from your reference 99.5:1). Adjust if different.
    private static final double CAROUSEL_GEAR_RATIO = 99.5;

    // Carousel encoder ticks per output shaft revolution
    private static final double CAROUSEL_TICKS_PER_REV = MOTOR_TICKS_PER_REV * CAROUSEL_GEAR_RATIO;

    // Servo angles: servo position mapping (0..1). We'll map -150..+150 deg into 0..1
    private static final double SERVO_MIN_ANGLE = -150.0;
    private static final double SERVO_MAX_ANGLE = 150.0;

    // Shooter servo shoot angles (rotate up 80 degrees clockwise, then back)
    private static final double SHOOTER_ROTATE_DEG = 80.0;

    // RPM presets
    private static final double RPM_STOP = 0.0;
    private static final double RPM_PRESET_1000 = 1000.0;
    private static final double RPM_PRESET_2500 = 2500.0;
    private static final double RPM_PRESET_5000 = 5000.0;

    // Velocity conversion helper: RPM -> ticks per second
    private double rpmToTicksPerSec(double rpm, double gearRatio) {
        double motorRevsPerMin = rpm / gearRatio;
        double motorRevsPerSec = motorRevsPerMin / 60.0;
        return motorRevsPerSec * MOTOR_TICKS_PER_REV;
    }

    // Servo position helper: map angle degrees to servo position 0..1
    private double angleToServoPosition(double angleDegrees) {
        double clipped = Math.max(SERVO_MIN_ANGLE, Math.min(SERVO_MAX_ANGLE, angleDegrees));
        return (clipped - SERVO_MIN_ANGLE) / (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE);
    }

    @Override
    public void runOpMode() {

        // init hardware
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        intake = hardwareMap.get(DcMotor.class, "intake");

        shooterLeft  = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");

        carousel = hardwareMap.get(DcMotorEx.class, "carousel");

        shooterServo = hardwareMap.get(Servo.class, "shooterServo");

        // Motor directions - adjust if needed
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        shooterLeft.setDirection(DcMotor.Direction.FORWARD);
        shooterRight.setDirection(DcMotor.Direction.FORWARD);

        // Carousel initial position: initialize to 0 degrees (set current encoder position as zero)
        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Servo initial (assume 0 degrees = center -> we define original position as 0 deg)
        shooterServo.setPosition(angleToServoPosition(0.0));

        boolean lowSpeedMode = false;
        double desiredShooterRPM = RPM_STOP;
        boolean shooterAtTarget = false;

        // Set motors to RUN_USING_ENCODER for velocity control possibility
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        ElapsedTime servoTimer = new ElapsedTime();
        boolean shootingSequenceActive = false;
        final double servoMoveDelay = 0.25; // seconds pause between move up and move down

        while (opModeIsActive()) {

            // ---------- DRIVER (gamepad1) controls - driving ----------
            // Left stick: translation (drive + strafe)
            double driveY = -gamepad1.left_stick_y; // up positive
            double driveX = gamepad1.left_stick_x;  // right positive

            // Right stick: rotation
            double turn = gamepad1.right_stick_x; // right positive -> clockwise

            // Low speed mode if RT pressed
            lowSpeedMode = gamepad1.right_trigger > 0.05; // RT threshold
            double speedFactor = lowSpeedMode ? LOW_SPEED_FACTOR : 1.0;

            // Mecanum drive mixing:
            // forward/back: driveY
            // strafe: driveX
            // rotation: turn
            double lf = driveY + driveX + turn;
            double lb = driveY - driveX + turn;
            double rf = driveY - driveX - turn;
            double rb = driveY + driveX - turn;

            // Scale down if any motor magnitude > 1
            double max = Math.max(Math.max(Math.abs(lf), Math.abs(lb)), Math.max(Math.abs(rf), Math.abs(rb)));
            if (max > 1.0) {
                lf /= max;
                lb /= max;
                rf /= max;
                rb /= max;
            }

            // Apply speed factor
            lf *= speedFactor;
            lb *= speedFactor;
            rf *= speedFactor;
            rb *= speedFactor;

            leftFront.setPower(-lf);
            leftBack.setPower(-lb);
            rightFront.setPower(-rf);
            rightBack.setPower(-rb);

            // ---------- OPERATOR (gamepad2) controls ----------
            // Shooter RPM presets (X, A, B, Y)
            if (gamepad2.x) {
                desiredShooterRPM = RPM_PRESET_1000;
            } else if (gamepad2.a) {
                desiredShooterRPM = RPM_PRESET_2500;
            } else if (gamepad2.b) {
                desiredShooterRPM = RPM_PRESET_5000;
            } else if (gamepad2.y) {
                desiredShooterRPM = RPM_STOP;
            }

            // Set shooter velocities using DcMotorEx setVelocity (ticks per second)
            double targetTicksPerSec = rpmToTicksPerSec(desiredShooterRPM, SHOOTER_GEAR_RATIO);
            // For zero RPM, set power 0
            if (desiredShooterRPM <= 0.5) {
                shooterLeft.setPower(0.0);
                shooterRight.setPower(0.0);
            } else {
                shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                shooterLeft.setVelocity(targetTicksPerSec);
                shooterRight.setVelocity(targetTicksPerSec);
            }

            // Check actual shooter RPMs using current velocity (ticks per sec -> RPM)
            double leftTicksPerSec = shooterLeft.getVelocity();
            double rightTicksPerSec = shooterRight.getVelocity();

            double leftRPM = (leftTicksPerSec / MOTOR_TICKS_PER_REV) * 60.0 * SHOOTER_GEAR_RATIO;
            double rightRPM = (rightTicksPerSec / MOTOR_TICKS_PER_REV) * 60.0 * SHOOTER_GEAR_RATIO;

            // Consider at target if within tolerance (e.g., +/- 5% or 50 RPM minimum)
            double toleranceRPM = Math.max(50.0, desiredShooterRPM * 0.05); // 5% or 50RPM
            shooterAtTarget = Math.abs(leftRPM - desiredShooterRPM) <= toleranceRPM
                    && Math.abs(rightRPM - desiredShooterRPM) <= toleranceRPM
                    && desiredShooterRPM > 0.5;

            // Shooter servo shoot action: RT on gamepad2
            if (gamepad2.right_trigger > 0.05 && !shootingSequenceActive) {
                // Start the shooting sequence: rotate servo up 80 deg then back
                shootingSequenceActive = true;
                servoTimer.reset();
                // Move up (clockwise): +80 degrees from original (we defined original 0 deg)
                shooterServo.setPosition(angleToServoPosition(SHOOTER_ROTATE_DEG));
            }

            if (shootingSequenceActive) {
                // Wait servoMoveDelay seconds then move back
                if (servoTimer.seconds() > servoMoveDelay && servoTimer.seconds() < (servoMoveDelay + 0.5)) {
                    shooterServo.setPosition(angleToServoPosition(0.0)); // back to 0
                }
                if (servoTimer.seconds() > (servoMoveDelay + 0.5)) {
                    shootingSequenceActive = false;
                }
            }

            // Carousel control:
            // LB + D-Pad R => rotate 60 deg right of current position
            // LB + D-Pad L => rotate 60 deg left of current position
            // D-Pad R => rotate 120 deg right
            // D-Pad L => rotate 120 deg left
            // We'll implement by reading dpad presses and commanding RUN_TO_POSITION stepwise.
            if (gamepad2.dpad_right || gamepad2.dpad_left || (gamepad2.left_bumper && (gamepad2.dpad_left || gamepad2.dpad_right))) {
                // Read current position
                int currentTicks = carousel.getCurrentPosition();
                double degPerOutputRev = 360.0;
                double ticksPerDegree = CAROUSEL_TICKS_PER_REV / degPerOutputRev;
                double degreesToMove = 0.0;

                if (gamepad2.left_bumper && gamepad2.dpad_right) {
                    degreesToMove = 60.0;
                } else if (gamepad2.left_bumper && gamepad2.dpad_left) {
                    degreesToMove = -60.0;
                } else if (!gamepad2.left_bumper && gamepad2.dpad_right) {
                    degreesToMove = 120.0;
                } else if (!gamepad2.left_bumper && gamepad2.dpad_left) {
                    degreesToMove = -120.0;
                }

                int ticksToMove = (int) Math.round(degreesToMove * ticksPerDegree);
                int target = currentTicks + ticksToMove;

                // Command RUN_TO_POSITION
                carousel.setTargetPosition(target);
                carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                carousel.setPower(0.5); // moderate speed
            } else {
                // allow manual stop if not running to position
                if (carousel.isBusy() && carousel.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                    // let it finish
                } else {
                    carousel.setPower(0.0);
                    // keep RUN_USING_ENCODER so we can read positions
                    carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
            }

            // Intake control example (operator): Right bumper to intake forward, left bumper to reverse
            if (gamepad2.right_bumper) {
                intake.setPower(1.0);
            } else if (gamepad2.left_trigger > 0.05) {
                intake.setPower(-1.0);
            } else {
                intake.setPower(0.0);
            }

            // Telemetry: only show RPMs of both shooter motors and whether both at desired RPM
            telemetry.clearAll();
            telemetry.addData("Desired RPM", "%.0f", desiredShooterRPM);
            telemetry.addData("ShooterLeft RPM", "%.1f", leftRPM);
            telemetry.addData("ShooterRight RPM", "%.1f", rightRPM);
            telemetry.addData("Both at target", shooterAtTarget);
            telemetry.addData("Low speed mode", lowSpeedMode);
            telemetry.update();

            idle();
        }
    }
}
