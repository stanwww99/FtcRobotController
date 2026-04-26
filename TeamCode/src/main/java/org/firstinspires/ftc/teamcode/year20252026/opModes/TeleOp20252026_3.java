package org.firstinspires.ftc.teamcode.year20252026.opModes;

import android.annotation.SuppressLint;
import android.graphics.Color;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.year20252026.control.*;

import java.util.Arrays;

@TeleOp(name = "TeleOp20252026_3", group = "TeleOp")
public class TeleOp20252026_3 extends LinearOpMode {

    // -----------------------------
    // DRIVE MOTORS (Control Hub)
    // -----------------------------
    private DcMotor leftFront, rightFront, leftBack, rightBack;

    // -----------------------------
    // MECHANISM SUBSYSTEMS
    // -----------------------------
    private Carousel carousel;     // Rotating ball selector
    private DcMotor intake;        // Tetrix intake motor
    private Shooter shooter;       // Flywheel shooter subsystem

    // -----------------------------
    // FIELD-ORIENTED DRIVE
    // -----------------------------
    private IMU imu;
    private boolean fieldDriveMode = true;

    // -----------------------------
    // CAROUSEL LOGICAL ANGLE
    // -----------------------------
    private int carouselAngleDeg = 0;

    // -----------------------------
    // INTAKE STATE MACHINE
    // -----------------------------
    private boolean intakeActive = false;
    private long currTimeIntake = 0;

    // -----------------------------
    // SHOOTER STATE MACHINE
    // -----------------------------
    private long currTimeShooter;

    // -----------------------------
    // COLOR SENSORS (FRONT + BACK)
    // -----------------------------
    private NormalizedColorSensor colorSensorFront;
    private NormalizedColorSensor colorSensorBack;

    // Mode 1 = intake pattern, Mode 2 = shooting pattern
    private int mode = 2;

    // Pattern arrays (3-ball carousel)
    private char[] colorIntake = {'a', 'a', 'a'};
    private char[] colorShoot  = {'a', 'a', 'a'};

    // Current detected colors
    private char front = 'a';
    private char back  = 'a';

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
                shooter.start(-1200);
                intakeActive = true;
            }

            doAll();

            // Stop shooter-intake when Y is released
            if (intakeActive && !gamepad1.y) {
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
            if (shooter.isShooterActive() && !intakeActive) {
                doAll();

                if (shooter.isTargetMet() && !shooter.isPusherActive()) {
                    shooter.push();
                    currTimeShooter = System.currentTimeMillis();

                    // Reset color arrays after shooting
                    colorShoot[1] = 'a';
                    colorIntake[1] = 'a';
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
    // SHOOTER BUTTON LOGIC (gamepad2)
    // -----------------------------
    private void getShooterControls() {

        if (!shooter.isShooterActive()) {
            if (gamepad2.x) shooter.start(1000);
            if (gamepad2.a) shooter.start(2500);
            if (gamepad2.b) shooter.start(4500);
        }

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

        carousel = new Carousel(hardwareMap, Carousel.MANUAL);
        intake   = hardwareMap.get(DcMotor.class, "intake");
        shooter  = new Shooter(hardwareMap);

        colorSensorFront = hardwareMap.get(NormalizedColorSensor.class, "colorSensorFront");
        colorSensorBack  = hardwareMap.get(NormalizedColorSensor.class, "colorSensorBack");

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

        telemetry.addData("Carousel Degree", carouselAngleDeg);

        if (mode == 1) {
            telemetry.addData("ColorFront", front);
            telemetry.addData("Intake Pattern (Front, Other1, Other2)", Arrays.toString(colorIntake));
        }

        if (mode == 2) {
            telemetry.addData("ColorBack", back);
            telemetry.addData("Shoot Pattern (Other1, Center, Other2)", Arrays.toString(colorShoot));
        }

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

            if (gamepad1.bWasPressed()) {
                imu.resetYaw();
            }

            double theta = Math.atan2(ly, lx);
            double r = Math.hypot(lx, ly);

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
        double speedLimit = gamepad1.right_trigger > 0.05 ? 0.5 : 1.0;

        leftFront.setPower(lf * speedLimit);
        rightFront.setPower(rf * speedLimit);
        leftBack.setPower(lb * speedLimit);
        rightBack.setPower(rb * speedLimit);
    }

    private double rangeClip(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    // -----------------------------
    // CAROUSEL AUTOMATIC ROTATION (gamepad2 D-pad)
    // -----------------------------
    private void autoCarouselControls() {

        int rotateDegree = 0;

        if (gamepad2.dpad_right) {
            rotateDegree = 120;
            carousel.rotateThirdRight();
        }
        if (gamepad2.dpad_left) {
            rotateDegree = -120;
            carousel.rotateThirdLeft();
        }
        if (gamepad2.dpad_up) {
            rotateDegree = 60;
            carousel.rotateSixthRight();
        }
        if (gamepad2.dpad_down) {
            rotateDegree = -60;
            carousel.rotateSixthLeft();
        }

        shift(rotateDegree);

        // -----------------------------
        // AUTO-ALIGN GREEN BALL (LB)
        // -----------------------------
        if (gamepad2.leftBumperWasPressed()) {

            if (mode == 1) {
                rotateDegree = 60;
                carousel.rotateSixthRight();
                shift(rotateDegree);
            }

            if (colorShoot[1] == 'g') {
                // already centered
            } else if (colorShoot[0] == 'g') {
                rotateDegree = 120;
                carousel.rotateThirdRight();
                shift(rotateDegree);
            } else if (colorShoot[2] == 'g') {
                rotateDegree = -120;
                carousel.rotateThirdLeft();
                shift(rotateDegree);
            }
        }

        // -----------------------------
        // AUTO-ALIGN PURPLE BALL (RB)
        // -----------------------------
        if (gamepad2.rightBumperWasPressed()) {

            if (mode == 1) {
                rotateDegree = 60;
                carousel.rotateSixthRight();
                shift(rotateDegree);
            }

            if (colorShoot[1] == 'p') {
                // already centered
            } else if (colorShoot[0] == 'p') {
                rotateDegree = 120;
                carousel.rotateThirdRight();
                shift(rotateDegree);
            } else if (colorShoot[2] == 'p') {
                rotateDegree = -120;
                carousel.rotateThirdLeft();
                shift(rotateDegree);
            }
        }
    }

    // -----------------------------
    // SHIFT PATTERN ARRAYS BASED ON ROTATION
    // -----------------------------
    private void shift(int rotateDegree) {

        // 120° rotation shifts array by one position
        if (rotateDegree == 120 && mode == 1) {
            shiftRightByOne(colorIntake);
        } else if (rotateDegree == 120 && mode == 2) {
            shiftRightByOne(colorIntake);
            shiftRightByOne(colorShoot);
        }

        if (rotateDegree == -120 && mode == 1) {
            shiftLeftByOne(colorIntake);
        } else if (rotateDegree == -120 && mode == 2) {
            shiftLeftByOne(colorShoot);
            shiftLeftByOne(colorIntake);
        }

        // 60° rotation switches between intake mode and shoot mode
        if (rotateDegree == 60 || rotateDegree == -60) {
            if (mode == 1) {
                mode = 2;
                colorShoot = Arrays.copyOf(colorIntake, 3);
            } else if (mode == 2) {
                mode = 1;
            }
        }

        carouselAngleDeg += rotateDegree;

        // Additional adjustments when crossing 120° boundaries
        if (rotateDegree == -60) {
            if (mode == 1 && carouselAngleDeg % 120 == 0) {
                shiftLeftByOne(colorIntake);
            } else if (mode == 2 && carouselAngleDeg % 120 == 0) {
                shiftLeftByOne(colorShoot);
            }
        } else if (rotateDegree == 60) {
            if (mode == 1 && carouselAngleDeg % 120 != 0) {
                shiftRightByOne(colorIntake);
            } else if (mode == 2 && carouselAngleDeg % 120 != 0) {
                shiftRightByOne(colorShoot);
                shiftRightByOne(colorIntake);
            }
        }
    }

    // -----------------------------
    // MANUAL CAROUSEL CONTROL (gamepad2 triggers)
    // -----------------------------
    private void manualCarouselControls() {
        double carouselPower = 0.0;

        if (gamepad2.right_trigger > 0.05) {
            carouselPower = -gamepad2.right_trigger / 2;
        } else if (gamepad2.left_trigger > 0.05) {
            carouselPower = gamepad2.left_trigger / 2;
        }

        carousel.move(carouselPower);
    }

    // -----------------------------
    // COLOR SENSOR READING
    // -----------------------------
    private void readColor() {

        NormalizedRGBA colorsfront = colorSensorFront.getNormalizedColors();
        float[] hsv1 = new float[3];
        Color.colorToHSV(colorsfront.toColor(), hsv1);

        NormalizedRGBA colorsBack = colorSensorBack.getNormalizedColors();
        float[] hsv2 = new float[3];
        Color.colorToHSV(colorsBack.toColor(), hsv2);

        // FRONT SENSOR (mode 1)
        if (isColorGreen(hsv1) && mode == 1) {
            front = 'g';
        } else if (isColorPurple(hsv1)) {
            front = 'p';
        } else {
            front = 'a';
        }

        // BACK SENSOR (mode 2)
        if (isColorGreen(hsv2) && mode == 2) {
            back = 'g';
        } else if (isColorPurple(hsv2) && mode == 2) {
            back = 'p';
        } else if (mode == 2) {
            back = 'a';
        }

        // Fill intake array only when first slot is empty
        if (mode == 1 && colorIntake[0] == 'a') {
            colorIntake[0] = front;
        }
    }

    private boolean isColorGreen(float[] hsv) {
        double hue = hsv[0];
        return (hue >= 100 && hue <= 180);
    }

    private boolean isColorPurple(float[] hsv) {
        double hue = hsv[0];
        return (hue >= 200 && hue <= 245);
    }

    // -----------------------------
    // ARRAY SHIFT HELPERS
    // -----------------------------
    public static void shiftRightByOne(char[] a) {
        int n = a.length;
        char last = a[n - 1];
        for (int i = n - 1; i > 0; i--) {
            a[i] = a[i - 1];
        }
        a[0] = last;
    }

    public static void shiftLeftByOne(char[] a) {
        int n = a.length;
        char first = a[0];
        for (int i = 0; i < n - 1; i++) {
            a[i] = a[i + 1];
        }
        a[n - 1] = first;
    }

    // -----------------------------
    // MAIN UPDATE LOOP
    // -----------------------------
    private void doAll() {
        shooter.updateRPM();
        readColor();
        updateTelemetry();
    }
}
