package org.firstinspires.ftc.teamcode.year20252026.opModes;

import android.annotation.SuppressLint;
import android.graphics.Color;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.year20252026.control.*;

@TeleOp(name = "TeleOp20252026_2", group = "TeleOp")
public class TeleOp20252026_2 extends LinearOpMode {

    // -----------------------------
    // DRIVE MOTORS (Control Hub)
    // -----------------------------
    private DcMotor leftFront, rightFront, leftBack, rightBack;

    // -----------------------------
    // MECHANISM SUBSYSTEMS
    // -----------------------------
    private Carousel carousel;   // Rotating ball selector (GoBilda 60 RPM motor)
    private DcMotor intake;      // Tetrix intake motor
    private Shooter shooter;     // Flywheel shooter subsystem

    // -----------------------------
    // FIELD-ORIENTED DRIVE
    // -----------------------------
    private IMU imu;
    private boolean fieldDriveMode = true;  // toggle with gamepad1.back
    private double startHeading;

    // -----------------------------
    // INTAKE STATE MACHINE
    // -----------------------------
    private boolean intakeActive = false;
    private long currTimeIntake = 0;

    // -----------------------------
    // SHOOTER STATE MACHINE
    // -----------------------------
    private long currTimeShooter = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        initHardware();

        telemetry.clear();
        telemetry.addLine("Ready. Press Play to start.");
        telemetry.update();

        waitForStart();

        // -----------------------------
        // MAIN TELEOP LOOP
        // -----------------------------
        while (opModeIsActive()) {

            // -----------------------------
            // DRIVE CONTROL (gamepad1)
            // -----------------------------
            drive();

            // -----------------------------
            // SHOOTER-AS-INTAKE (gamepad1 Y)
            // Reverse shooter motor to suck in balls
            // -----------------------------
            if (gamepad1.y && !intakeActive) {
                shooter.start(-1200);  // negative RPM = reverse intake
                intakeActive = true;
                currTimeIntake = System.currentTimeMillis();
            }

            doAll();

            // Stop shooter-intake after 1.75 seconds
            if (intakeActive && System.currentTimeMillis() - currTimeIntake >= 1750) {
                intakeActive = false;
                shooter.stop();
            }

            doAll();

            // -----------------------------
            // REAL INTAKE MOTOR (gamepad1)
            // -----------------------------
            double intakePower = 0.0;

            if (gamepad1.left_bumper) {
                intakePower = -1.0;  // reverse full speed
            } else if (gamepad1.left_trigger > 0.05) {
                intakePower = gamepad1.left_trigger;  // analog forward
            }

            // A button = slow mode (30%)
            if (gamepad1.a) intakePower *= 0.30;

            intake.setPower(intakePower);

            // Toggle field-oriented drive
            if (gamepad1.backWasReleased())
                fieldDriveMode = !fieldDriveMode;

            // -----------------------------
            // SHOOTER RPM CONTROL (gamepad2)
            // -----------------------------
            doAll();
            getShooterControls();
            doAll();

            // When shooter reaches target RPM → fire servo
            if (shooter.isShooterActive()) {
                doAll();

                if (shooter.isTargetMet() && !shooter.isPusherActive()) {
                    shooter.push();
                    currTimeShooter = System.currentTimeMillis();
                }
            }

            doAll();

            // Retract servo after 200 ms
            if (shooter.isPusherActive() &&
                    System.currentTimeMillis() - currTimeShooter >= 200) {
                shooter.stop();
            }

            doAll();

            // -----------------------------
            // CAROUSEL CONTROL (gamepad2)
            // -----------------------------
            if (!carousel.isRotateActive()) {
                manualCarouselControls();
                autoCarouselControls();
            }

            // Stop carousel rotation when finished or cancelled
            if (carousel.isFinished() ||
                    (carousel.isRotateActive() && gamepad2.back)) {
                carousel.stop();
            }

            doAll();
        }
    }

    // -----------------------------
    // CAROUSEL AUTOMATIC ROTATION (gamepad2 D-pad)
    // -----------------------------
    private void autoCarouselControls() {
        if (gamepad2.dpad_right) {
            carousel.rotateThirdRight();  // +120°
        }
        if (gamepad2.dpad_left) {
            carousel.rotateThirdLeft();   // -120°
        }
        if (gamepad2.dpad_up) {
            carousel.rotateSixthRight();  // +60°
        }
        if (gamepad2.dpad_down) {
            carousel.rotateSixthLeft();   // -60°
        }
    }

    // -----------------------------
    // CAROUSEL MANUAL CONTROL (gamepad2 triggers)
    // -----------------------------
    private void manualCarouselControls() {
        double carouselPower = 0.0;

        if (gamepad2.right_trigger > 0.05) {
            carouselPower = -gamepad2.right_trigger / 2;  // reverse
        } else if (gamepad2.left_trigger > 0.05) {
            carouselPower = gamepad2.left_trigger / 2;    // forward
        }

        carousel.move(carouselPower);
    }

    // -----------------------------
    // SHOOTER BUTTON LOGIC (gamepad2)
    // -----------------------------
    private void getShooterControls() {

        // Only allow new RPM commands if shooter is not already active
        if (!shooter.isShooterActive()) {
            if (gamepad2.x) shooter.start(1000);
            if (gamepad2.a) shooter.start(2500);
            if (gamepad2.b) shooter.start(4500);
        }

        // Stop shooter
        if (gamepad2.y) shooter.stop();
    }

    // -----------------------------
    // HARDWARE INITIALIZATION
    // -----------------------------
    private void initHardware() {

        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        // Retrieve IMU from MyGyro (shared between Auto + TeleOp)
        if ((imu = MyGyro.imu) == null) {
            imu = MyGyro.createIMU(hardwareMap);
        }

        // Carousel starts in MANUAL mode
        carousel = new Carousel(hardwareMap, Carousel.MANUAL);

        intake   = hardwareMap.get(DcMotor.class, "intake");
        shooter  = new Shooter(hardwareMap);

        // Drive motor directions
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // Brake mode for precise stopping
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Intake defaults
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // -----------------------------
    // TELEMETRY
    // -----------------------------
    @SuppressLint("DefaultLocale")
    private void updateTelemetry() {
        telemetry.clearAll();
        telemetry.addData("Current Shooter RPM", shooter.getCurrentRPM());
        telemetry.addData("Target RPM", String.format("%.1f", shooter.getTargetRPM()));
        telemetry.addData("Shooter Amps", shooter.getMotor().getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Target RPM Met?", shooter.isTargetMet());

        telemetry.addData("Carousel Position", carousel.getMotor().getCurrentPosition());
        telemetry.addData("Carousel Power", carousel.getMotor().getPower());
        telemetry.addData("Carousel Active", carousel.isRotateActive());
        telemetry.addData("Last Position", carousel.getPosition());
        telemetry.addData("Carousel Delta",
                carousel.getMotor().getCurrentPosition() - carousel.getPosition());
        telemetry.addData("Carousel Amps",
                carousel.getMotor().getCurrent(CurrentUnit.AMPS));

        telemetry.update();
    }

    // -----------------------------
    // DRIVE SYSTEM (MECANUM + FIELD ORIENTED)
    // -----------------------------
    private void drive() {

        double lx = gamepad1.left_stick_x;     // strafe
        double ly = -gamepad1.left_stick_y;    // forward/back
        double rx = -gamepad1.right_stick_x;   // rotation

        // FIELD-ORIENTED DRIVE
        if (fieldDriveMode) {

            // Reset heading (gamepad1 B)
            if (gamepad1.bWasPressed()) {
                imu.resetYaw();
            }

            // Convert joystick vector into robot-centric frame
            double theta = Math.atan2(ly, lx);
            double r = Math.hypot(lx, ly);

            // Subtract robot heading from joystick direction
            theta = AngleUnit.normalizeRadians(
                    theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)
            );

            ly = r * Math.sin(theta);
            lx = r * Math.cos(theta);
        }

        // Mecanum mixing
        double lf = ly + lx + rx;
        double rf = ly - lx - rx;
        double lb = ly - lx + rx;
        double rb = ly + lx - rx;

        // Normalize and invert
        lf = rangeClip(-lf, -1.0, 1.0);
        rf = rangeClip(-rf, -1.0, 1.0);
        lb = rangeClip(-lb, -1.0, 1.0);
        rb = rangeClip(-rb, -1.0, 1.0);

        // Slow mode (RT)
        double speedLimit = gamepad1.right_trigger > 0.05 ? 0.30 : 1.0;

        leftFront.setPower(lf * speedLimit);
        rightFront.setPower(rf * speedLimit);
        leftBack.setPower(lb * speedLimit);
        rightBack.setPower(rb * speedLimit);
    }

    private double rangeClip(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private void doAll() {
        shooter.updateRPM();
        updateTelemetry();
    }
}
