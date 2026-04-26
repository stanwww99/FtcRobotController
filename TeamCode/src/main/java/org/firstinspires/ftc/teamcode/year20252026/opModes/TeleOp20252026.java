package org.firstinspires.ftc.teamcode.year20252026.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "TeleOp20252026", group = "TeleOp")
public class TeleOp20252026 extends LinearOpMode {

    // -----------------------------
    // DRIVE MOTORS (Control Hub)
    // -----------------------------
    private DcMotor leftFront, rightFront, leftBack, rightBack;

    // -----------------------------
    // MECHANISM MOTORS (Expansion Hub)
    // -----------------------------
    private DcMotorEx carousel;      // GoBilda 60 RPM motor (99.5:1 gearbox)
    private DcMotor intake;          // Tetrix intake motor
    private DcMotorEx shooter;       // GoBilda 6000 RPM flywheel motor

    // -----------------------------
    // SERVO (Smart Servo Pusher)
    // -----------------------------
    private Servo pusher;

    // -----------------------------
    // SHOOTER RPM TRACKING
    // -----------------------------
    private double currentRPM = 0.0;
    private double targetRPM = 0.0;
    private boolean targetMet = false;

    // Shooter encoder resolution (GoBilda 5202 motor)
    private static final double SHOOTER_PPR = 28.0;

    // Carousel encoder resolution (GoBilda 99.5:1 gearbox)
    private static final double CAROUSEL_PPR3rd = 2786.2 / 3;
    private static final double CAROUSEL_PPR6th = 2786.2 / 6;

    // Servo range mapping (0–300 degrees → 0.0–1.0)
    private static final double SERVO_FULL_RANGE_DEG = 300.0;

    // Carousel angle tracking (logical angle)
    private int carouselAngleDeg = 0;

    // -----------------------------
    // INTAKE STATE MACHINE
    // -----------------------------
    private boolean intakeActive = false;
    private long currTimeIntake = 0;

    // -----------------------------
    // SHOOTER STATE MACHINE
    // -----------------------------
    private boolean servoActive = false;
    private boolean rpmShooterHold = false;
    private boolean shooterActive = false;
    private long currTimeShooter = 0;

    // -----------------------------
    // CAROUSEL STATE MACHINE
    // -----------------------------
    private boolean rotateActive = false;
    private long currPos = 0;
    private double required = 0;

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
            // INTAKE USING SHOOTER MOTOR (gamepad1 Y)
            // -----------------------------
            if (gamepad1.y && !intakeActive) {
                double RPM = -1200;  // reverse shooter to act as intake
                double ticksPerSec = RPM * SHOOTER_PPR / 60.0;

                intakeActive = true;
                shooter.setVelocity(ticksPerSec);
                currTimeIntake = System.currentTimeMillis();
            }

            doAll();

            // Stop intake after 1.75 seconds
            if (intakeActive && System.currentTimeMillis() - currTimeIntake >= 1750) {
                intakeActive = false;
                shooter.setVelocity(0);
            }

            doAll();

            // -----------------------------
            // REAL INTAKE MOTOR CONTROL (gamepad1)
            // -----------------------------
            double intakePower = 0.0;

            if (gamepad1.left_bumper) {
                intakePower = -1.0;  // reverse full speed
            } else if (gamepad1.left_trigger > 0.05) {
                intakePower = gamepad1.left_trigger;  // analog forward
            }

            // A button limits intake to 30%
            if (gamepad1.a) intakePower *= 0.30;

            intake.setPower(intakePower);

            // -----------------------------
            // SHOOTER RPM CONTROL (gamepad2)
            // -----------------------------
            doAll();

            if (gamepad2.x && !shooterActive) {
                setDcMotorRPM(1000);
                shooterActive = true;
                servoActive = false;
            }
            if (gamepad2.a && !shooterActive) {
                setDcMotorRPM(2500);
                shooterActive = true;
                servoActive = false;
            }
            if (gamepad2.b && !shooterActive) {
                setDcMotorRPM(4500);
                shooterActive = true;
                servoActive = false;
            }

            // Stop shooter (gamepad2 Y)
            if (gamepad2.y && shooterActive) {
                shooterActive = false;
                stopDcMotor();
            }

            doAll();

            // -----------------------------
            // SHOOTER PUSHER SERVO LOGIC
            // -----------------------------
            if (shooterActive) {
                doAll();

                // When RPM reaches target → fire servo
                if (targetMet && !rpmShooterHold) {
                    rpmShooterHold = true;
                    servoActive = true;
                    currTimeShooter = System.currentTimeMillis();
                    setServoAngle(pusher, 80.0);  // push ball
                }
            }

            doAll();

            // Retract servo after 200 ms
            if (servoActive && System.currentTimeMillis() - currTimeShooter >= 200) {
                setServoAngle(pusher, 0.0);
                stopDcMotor();
                shooterActive = false;
                rpmShooterHold = false;
                servoActive = false;
            }

            doAll();

            // -----------------------------
            // CAROUSEL MANUAL CONTROL (gamepad2 triggers)
            // -----------------------------
            double carouselPower = 0.0;

            if (gamepad2.right_trigger > 0.05 && !rotateActive) {
                carouselPower = -gamepad2.right_trigger;
            } else if (gamepad2.left_trigger > 0.05 && !rotateActive) {
                carouselPower = gamepad2.left_trigger;
            }

            // Right bumper = slow mode
            if (gamepad2.right_bumper) {
                carouselPower *= 0.3;
            }

            if (!rotateActive) {
                carousel.setPower(carouselPower);
            }

            // -----------------------------
            // CAROUSEL PRECISE ROTATION (gamepad2 D-pad)
            // -----------------------------
            int rotateDegree = 0;

            if (gamepad2.dpad_right && !rotateActive) {
                rotateDegree = 120;
                rotateActive = true;
                currPos = carousel.getCurrentPosition();
                required = CAROUSEL_PPR3rd;
                carousel.setPower(0.25);
            }
            if (gamepad2.dpad_left && !rotateActive) {
                rotateDegree = -120;
                rotateActive = true;
                currPos = carousel.getCurrentPosition();
                required = CAROUSEL_PPR3rd;
                carousel.setPower(-0.25);
            }
            if (gamepad2.dpad_up && !rotateActive) {
                rotateDegree = 60;
                rotateActive = true;
                required = CAROUSEL_PPR6th;
                currPos = carousel.getCurrentPosition();
                carousel.setPower(0.25);
            }
            if (gamepad2.dpad_down && !rotateActive) {
                rotateDegree = -60;
                rotateActive = true;
                required = CAROUSEL_PPR6th;
                currPos = carousel.getCurrentPosition();
                carousel.setPower(-0.25);
            }

            // Stop rotation when encoder reaches target
            if (rotateActive) {
                int offset = 500; // safety margin
                if (Math.abs(carousel.getCurrentPosition() - currPos) >= required - offset) {
                    required = 0;
                    rotateActive = false;
                    carousel.setPower(0);
                }
            }

            // Emergency cancel (LB)
            if (rotateActive && gamepad2.left_bumper) {
                rotateActive = false;
                required = 0;
                carousel.setPower(0);
            }

            // Update logical angle
            carouselAngleDeg += rotateDegree;
            carouselAngleDeg = normalizeAngle(carouselAngleDeg);

            doAll();
        }
    }

    // -----------------------------
    // HARDWARE INITIALIZATION
    // -----------------------------
    private void initHardware() {

        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        carousel = hardwareMap.get(DcMotorEx.class, "carousel");
        intake   = hardwareMap.get(DcMotor.class, "intake");
        shooter  = hardwareMap.get(DcMotorEx.class, "shooter");

        pusher = hardwareMap.get(Servo.class, "pusher");

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

        // Shooter motor setup
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Initialize pusher servo
        setServoAngle(pusher, 0);

        // Carousel setup
        carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        carouselAngleDeg = 0;
    }

    // -----------------------------
    // TELEMETRY
    // -----------------------------
    private void updateTelemetry() {
        telemetry.clearAll();
        telemetry.addData("Carousel Degree", carouselAngleDeg);
        telemetry.addData("Current Shooter RPM", currentRPM);
        telemetry.addData("Current Target RPM", String.format("%.1f", targetRPM));
        telemetry.addData("Shooter Amperage", shooter.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Target RPM Met?", targetMet);
        telemetry.addData("Carousel Position", carousel.getCurrentPosition());
        telemetry.addData("Carousel Power", carousel.getPower());
        telemetry.addData("Carousel Active", rotateActive);
        telemetry.addData("Last Position", currPos);
        telemetry.addData("Carousel Delta", carousel.getCurrentPosition() - currPos);
        telemetry.addData("Carousel Required", required);
        telemetry.addData("Carousel AMP", carousel.getCurrent(CurrentUnit.AMPS));
        telemetry.update();
    }

    // -----------------------------
    // DRIVE SYSTEM (MECANUM)
    // -----------------------------
    private void drive() {

        double lx = gamepad1.left_stick_x;     // strafe
        double ly = -gamepad1.left_stick_y;    // forward/back
        double rx = -gamepad1.right_stick_x;   // rotation

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

        // Low-speed mode (RT)
        double speedLimit = gamepad1.right_trigger > 0.05 ? 0.30 : 1.0;

        leftFront.setPower(lf * speedLimit);
        rightFront.setPower(rf * speedLimit);
        leftBack.setPower(lb * speedLimit);
        rightBack.setPower(rb * speedLimit);
    }

    // -----------------------------
    // SERVO CONTROL
    // -----------------------------
    private void setServoAngle(Servo s, double angleDeg) {
        double pos = rangeClip(angleDeg / SERVO_FULL_RANGE_DEG, 0.0, 1.0);
        s.setPosition(pos);
    }

    // -----------------------------
    // UTILITY FUNCTIONS
    // -----------------------------
    private double rangeClip(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private void setDcMotorRPM(double desiredRPM) {
        targetRPM = desiredRPM;
        double ticksPerSec = desiredRPM * SHOOTER_PPR / 60.0;
        shooter.setVelocity(ticksPerSec);
        doAll();
    }

    private void stopDcMotor() {
        setDcMotorRPM(0);
    }

    private void updateDcMotorRPM() {
        double ticksPerSec = shooter.getVelocity();
        currentRPM = ticksPerSec * 60.0 / SHOOTER_PPR;
        targetMet = currentRPM >= targetRPM;
    }

    private int normalizeAngle(int a) {
        int v = a % 360;
        if (v < 0) v += 360;
        return v;
    }

    private void doAll() {
        updateDcMotorRPM();
        updateTelemetry();
    }
}
