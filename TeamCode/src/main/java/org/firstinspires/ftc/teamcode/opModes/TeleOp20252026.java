package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOp20252026", group = "TeleOp")
public class TeleOp20252026 extends LinearOpMode {

    // Drive motors (control hub)
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;

    // Other motors (expansion or control hub as configured)
    private DcMotorEx intakeMotor;      // analog control with LB
    private DcMotorEx shooterMotor;     // velocity control via encoder
    private DcMotorEx carouselMotor;    // position-step using encoder

    // Standard servo used as pusher (smart servo in standard mode programmed as servo)
    private Servo pusherServo;

    // State variables
    private boolean lowDriveMode = false;         // RT pressed on driver gamepad
    private boolean intakeLowMode = false;        // A on driver reduces intake max to 30%
    private double shooterTargetRPM = 0.0;
    private boolean shooterAtTarget = false;

    // Constants and tuning
    private static final double DRIVE_MAX_POWER = 1.0;
    private static final double DRIVE_LOW_POWER = 0.30;
    private static final double INTAKE_MAX_POWER = 1.0;
    private static final double INTAKE_LOW_POWER = 0.30;
    private static final double PUSHER_UP_ANGLE = 0.444;   // normalized servo position for +80 degrees
    private static final double PUSHER_DOWN_ANGLE = 0.0;   // initial position

    // Carousel step in degrees
    private static final double CAROUSEL_STEP_SMALL_DEG = 60.0;
    private static final double CAROUSEL_STEP_LARGE_DEG = 120.0;

    // Encoder ticks per revolution for the shooter/carousel gearmotor (team should verify)
    // Common GoBILDA motors use 28 ticks per rev on their encoders; change if your encoder differs.
    private static final double TICKS_PER_REV = 28.0;

    // Helper to convert encoder velocity (ticks/sec) to RPM
    private double ticksPerSecToRPM(double ticksPerSec) {
        return (ticksPerSec / TICKS_PER_REV) * 60.0;
    }

    @Override
    public void runOpMode() {
        // Hardware map
        leftFront   = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack    = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront  = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack   = hardwareMap.get(DcMotorEx.class, "rightBack");

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        carouselMotor = hardwareMap.get(DcMotorEx.class, "carousel");

        pusherServo = hardwareMap.get(Servo.class, "smartServo");

        // Motor directions and modes - adjust directions if robot moves opposite
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);
        carouselMotor.setDirection(DcMotor.Direction.FORWARD);

        // Use encoders for shooter and carousel for velocity/position control
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        carouselMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize pusher servo to down position
        pusherServo.setPosition(PUSHER_DOWN_ANGLE);

        // Initialize carousel to 0 degrees (assumes 0 encoder position is desired origin)
        carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carouselMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        ElapsedTime pushTimer = new ElapsedTime();
        boolean pusherActive = false;

        while (opModeIsActive()) {
            // ----- DRIVER (gamepad1) controls -----
            Gamepad g1 = gamepad1;
            // Low drive mode when RT pressed
            lowDriveMode = g1.right_trigger > 0.05;

            double maxDrive = lowDriveMode ? DRIVE_LOW_POWER : DRIVE_MAX_POWER;

            // Left stick for translation (forward/back and strafe)
            double driveY = -g1.left_stick_y;   // forward is negative on stick, invert
            double driveX = g1.left_stick_x;    // strafe
            // Right stick X for rotation (spin)
            double turn  = g1.right_stick_x;

            // Scale by stick magnitude to allow analog control
            double leftFrontPower  = driveY + driveX + turn;
            double leftBackPower   = driveY - driveX + turn;
            double rightFrontPower = driveY - driveX - turn;
            double rightBackPower  = driveY + driveX - turn;

            // Normalize and scale
            double maxRaw = Math.max(
                    Math.max(Math.abs(leftFrontPower), Math.abs(leftBackPower)),
                    Math.max(Math.abs(rightFrontPower), Math.abs(rightBackPower))
            );
            if (maxRaw < 1e-6) maxRaw = 1.0;
            leftFrontPower  = Range.clip(leftFrontPower  / maxRaw * maxDrive, -maxDrive, maxDrive);
            leftBackPower   = Range.clip(leftBackPower   / maxRaw * maxDrive, -maxDrive, maxDrive);
            rightFrontPower = Range.clip(rightFrontPower / maxRaw * maxDrive, -maxDrive, maxDrive);
            rightBackPower  = Range.clip(rightBackPower  / maxRaw * maxDrive, -maxDrive, maxDrive);

            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightFront.setPower(rightFrontPower);
            rightBack.setPower(rightBackPower);

            // Intake behavior
            intakeLowMode = g1.a; // hold A to limit intake to 30%
            double intakePower = 0.0;

            if (g1.left_trigger > 0.05) {
                // LT held: intake backwards at full power (reverse)
                intakePower = -INTAKE_MAX_POWER;
            } else if (g1.left_bumper) {
                // LB held: forward; speed proportional to how hard LB is pressed is not analog on standard gamepad,
                // so use full power; if using an analog trigger for LB in a custom controller, change as needed.
                // Some controllers map left_bumper as boolean, left_trigger as analog.
                intakePower = INTAKE_MAX_POWER;
            } else {
                intakePower = 0.0;
            }

            // If A is held, limit intake magnitude
            if (intakeLowMode) {
                intakePower = Range.clip(intakePower, -INTAKE_LOW_POWER, INTAKE_LOW_POWER);
            }

            intakeMotor.setPower(intakePower);

            // ----- OPERATOR (gamepad2) controls -----
            Gamepad g2 = gamepad2;

            // Shooter RPM presets
            if (g2.x) {
                shooterTargetRPM = 1000.0;
                setShooterPowerForTarget(shooterTargetRPM);
            } else if (g2.a) {
                shooterTargetRPM = 2500.0;
                setShooterPowerForTarget(shooterTargetRPM);
            } else if (g2.b) {
                shooterTargetRPM = 5000.0;
                setShooterPowerForTarget(shooterTargetRPM);
            } else if (g2.y) {
                shooterTargetRPM = 0.0;
                shooterMotor.setPower(0.0);
            }

            // Shoot pulse with RT: servo rotates up 80 degrees then back
            if (g2.right_trigger > 0.1 && !pusherActive) {
                pusherActive = true;
                pusherServo.setPosition(PUSHER_UP_ANGLE);
                pushTimer.reset();
            }
            if (pusherActive) {
                // Wait 300 ms then retract (timing can be tuned)
                if (pushTimer.milliseconds() > 300) {
                    pusherServo.setPosition(PUSHER_DOWN_ANGLE);
                }
                if (pushTimer.milliseconds() > 600) {
                    pusherActive = false;
                }
            }

            // Carousel rotation commands using encoder-based position increments
            // We'll compute desired change and set target using RUN_TO_POSITION style behavior
            boolean lbPressed = g2.left_bumper;
            boolean dpadRight = g2.dpad_right;
            boolean dpadLeft = g2.dpad_left;

            if (lbPressed && dpadRight) {
                rotateCarouselByDegrees(CAROUSEL_STEP_SMALL_DEG);
            } else if (lbPressed && dpadLeft) {
                rotateCarouselByDegrees(-CAROUSEL_STEP_SMALL_DEG);
            } else if (dpadRight) {
                rotateCarouselByDegrees(CAROUSEL_STEP_LARGE_DEG);
            } else if (dpadLeft) {
                rotateCarouselByDegrees(-CAROUSEL_STEP_LARGE_DEG);
            }

            // Shooter closed-loop-ish monitoring: estimate RPM from encoder velocity
            double shooterVelocityTicksPerSec = shooterMotor.getVelocity(); // ticks/sec for DcMotorEx
            double currentShooterRPM = ticksPerSecToRPM(shooterVelocityTicksPerSec);
            // Simple on-target logic: within 3% tolerance
            if (Math.abs(shooterTargetRPM) < 1.0) {
                shooterAtTarget = false;
            } else {
                shooterAtTarget = Math.abs(currentShooterRPM - shooterTargetRPM) <= shooterTargetRPM * 0.03;
            }

            // Simple power control to reach target RPM: proportional controller on power
            if (shooterTargetRPM > 0.0) {
                // Convert current RPM to an error and correct power
                double error = shooterTargetRPM - currentShooterRPM;
                // Proportional gain - tune as necessary
                double kP = 0.00025; // small since RPM ranges large; tune on robot
                double currentPower = shooterMotor.getPower();
                double newPower = Range.clip(currentPower + error * kP, 0.0, 1.0);
                // If operator holds A on operator gamepad's RT to shoot we don't change power; but spec wants RT to trigger servo only.
                shooterMotor.setPower(newPower);
            }

            // Telemetry: only shooter RPM and whether at target
            telemetry.clearAll();
            telemetry.addData("ShooterRPM", "%.1f", currentShooterRPM);
            telemetry.addData("AtTarget", shooterAtTarget);
            telemetry.update();
        } // while opMode active
    }

    // Helper: map desired degrees rotation to encoder ticks for carousel and run small incremental move
    private void rotateCarouselByDegrees(double degrees) {
        // Convert degrees to ticks: degrees/360 * ticksPerRev for gearbox output shaft
        // If motor has gearbox, you must account for gear ratio. The team should update GEAR_RATIO accordingly.
        double GEAR_RATIO = 1.0; // set to gearbox ratio if known (e.g., 99.5 for a 99.5:1 gearbox)
        double ticksPerRevEffective = TICKS_PER_REV / GEAR_RATIO;
        int deltaTicks = (int) Math.round((degrees / 360.0) * ticksPerRevEffective);

        int current = carouselMotor.getCurrentPosition();
        int target = current + deltaTicks;

        // Run to position
        carouselMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        carouselMotor.setTargetPosition(target);
        carouselMotor.setPower(0.35); // low power for fine movement

        // Wait until position reached or timeout (non-blocking used minimally)
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && carouselMotor.isBusy() && t.milliseconds() < 1000) {
            // wait a short time for move to finish - keep OpMode responsive
            idle();
        }

        // Return to run using encoder for next commands
        carouselMotor.setPower(0.0);
        carouselMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Helper to set an initial shooter motor power proportional to desired RPM
    private void setShooterPowerForTarget(double rpm) {
        if (rpm <= 0.0) {
            shooterMotor.setPower(0.0);
            return;
        }
        // crude mapping; tune on robot
        double mappedPower = Range.clip(rpm / 6000.0, 0.0, 1.0); // if motor max ~6000rpm
        shooterMotor.setPower(mappedPower);
    }

}
