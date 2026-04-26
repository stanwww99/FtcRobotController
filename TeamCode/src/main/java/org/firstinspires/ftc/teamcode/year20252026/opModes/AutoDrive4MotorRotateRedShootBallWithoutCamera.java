package org.firstinspires.ftc.teamcode.year20252026.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.year20252026.control.*;

// Red Alliance autonomous WITHOUT camera.
// Uses fixed carousel rotation pattern to shoot 3 balls.
@Autonomous(name="AutoRedShoot3BallNoCamera", group="Autonomous")
public class AutoDrive4MotorRotateRedShootBallWithoutCamera extends LinearOpMode {

    // Delay before autonomous begins (ms)
    private int startDelay = 0;

    // Delay before backing up (ms)
    private int delay = 0;

    // 4‑motor drivetrain
    private DcMotor leftFront, leftBack, rightFront, rightBack;

    // Mechanisms
    private DcMotor intake;
    private Shooter shooter;     // Flywheel shooter subsystem
    private Carousel carousel;   // Rotating ball selector

    @Override
    public void runOpMode() throws InterruptedException {

        initHardware();

        telemetry.clearAll();
        telemetry.addLine("Ready. Press Play to start.");
        telemetry.update();

        waitForStart();
        telemetry.clearAll();
        telemetry.update();

        if (isStopRequested()) {
            return;
        }

        // -----------------------------
        // INITIAL DELAY PHASE
        // Allows alliance partners to move first
        // -----------------------------
        long start = System.currentTimeMillis();

        while (opModeIsActive()
                && (System.currentTimeMillis() - start) < startDelay) {
            mainDo();
        }

        // -----------------------------
        // DRIVE FORWARD TO SHOOTING POSITION
        // -----------------------------
        driveForwardFixedTime(0.7, 1);
        sleep(200);

        // -----------------------------
        // SHOOT 3 BALLS USING FIXED CAROUSEL ROTATION
        // No camera → rely on predetermined rotation pattern
        // -----------------------------

        // FIRST SHOT
        shoot(2500);

        // Rotate carousel 120° left (one-third turn)
        carousel.rotateThirdLeft();
        while (!carousel.isFinished() && opModeIsActive()) {
            sleep(50);
        }

        // SECOND SHOT
        shoot(2500);

        // Rotate carousel again
        carousel.rotateThirdLeft();
        while (!carousel.isFinished() && opModeIsActive()) {
            sleep(50);
        }

        // THIRD SHOT
        shoot(2500);

        sleep(200);

        // -----------------------------
        // ROTATE ROBOT TO EXIT PATH
        // Red alliance rotates opposite direction of blue
        // -----------------------------
        rotateFixedTime(0.5, -1);
        sleep(200);

        // -----------------------------
        // DELAY BEFORE BACKING UP
        // -----------------------------
        start = System.currentTimeMillis();

        while (opModeIsActive()
                && (System.currentTimeMillis() - start) < delay) {
            mainDo();
        }

        // -----------------------------
        // DRIVE BACKWARD TO PARKING ZONE
        // -----------------------------
        driveForwardFixedTime(1.4, 1);
        stopDrive();

        // -----------------------------
        // END PHASE — robot stands still
        // -----------------------------
        while (opModeIsActive()) {
            // No camera → nothing to update
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

        carousel = new Carousel(hardwareMap, Carousel.AUTO);
        intake   = hardwareMap.get(DcMotor.class, "intake");
        shooter  = new Shooter(hardwareMap);

        // Motor directions depend on wiring + drivetrain orientation
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // Brake mode ensures precise stopping
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Intake defaults
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // -----------------------------
    // DRIVE FORWARD FOR FIXED TIME
    // -----------------------------
    private void driveForwardFixedTime(double seconds, double power) {

        long start = System.currentTimeMillis();
        setDrivePower(power);

        while (opModeIsActive() && !isStopRequested()
                && (System.currentTimeMillis() - start) < (long)(seconds * 1000)) {
            // No telemetry needed during simple timed drive
        }

        stopDrive();
    }

    // Set all drive motors to same power
    private void setDrivePower(double p) {
        leftFront.setPower(p);
        leftBack.setPower(p);
        rightFront.setPower(p);
        rightBack.setPower(p);
    }

    // -----------------------------
    // ROTATE ROBOT FOR FIXED TIME
    // -----------------------------
    private void setRotatePower(double p) {
        leftFront.setPower(p);
        rightFront.setPower(-p);
        leftBack.setPower(p);
        rightBack.setPower(-p);
    }

    // CCW = negative, CW = positive
    private void rotateFixedTime(double seconds, double power) {

        long start = System.currentTimeMillis();
        setRotatePower(power);

        while (opModeIsActive() && !isStopRequested()
                && (System.currentTimeMillis() - start) < (long)(seconds * 1000)) {
            // Simple timed rotation
        }

        stopDrive();
    }

    // Stop robot
    private void stopDrive() {
        setDrivePower(0.0);
    }

    // -----------------------------
    // SHOOT ONE BALL AT TARGET RPM
    // -----------------------------
    private void shoot(double RPM) {

        shooter.start(RPM);

        // Wait until flywheel reaches target RPM
        while (opModeIsActive() && !shooter.isTargetMet()) {
            mainDo();
        }

        // Push ball into flywheel
        shooter.push();

        // Allow ball to clear
        long currMilli = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - currMilli < 300) {
            mainDo();
        }

        shooter.stop();
    }

    // -----------------------------
    // MAIN UPDATE LOOP
    // -----------------------------
    private void mainDo() {
        shooter.updateRPM();
        updateTelemetry();
    }

    // -----------------------------
    // TELEMETRY
    // -----------------------------
    private void updateTelemetry() {
        telemetry.clearAll();
        telemetry.addData("Shooter speed (ticks/sec): ", shooter.getMotor().getVelocity());
        telemetry.update();
    }
}
