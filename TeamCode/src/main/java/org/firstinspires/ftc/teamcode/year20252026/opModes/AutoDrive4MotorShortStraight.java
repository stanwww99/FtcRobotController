package org.firstinspires.ftc.teamcode.year20252026.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

@Autonomous(name="AutoDriveStraight", group="Autonomous")
public class AutoDrive4MotorShortStraight extends LinearOpMode {

    // Delay before driving forward (ms)
    // Used for timed autonomous routines where robot must wait before moving
    int delay = 24500;

    // -----------------------------
    // DRIVE MOTORS (4‑motor drivetrain)
    // -----------------------------
    private DcMotor leftFront, leftBack, rightFront, rightBack;

    // -----------------------------
    // VISION SYSTEM (AprilTag)
    // -----------------------------
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // AprilTag data storage
    private final HashMap<Integer, Double> idToDistanceMeters = new HashMap<>();
    private final List<Integer> seenTagIds = new ArrayList<>();

    // Pattern determined by AprilTag ID (g/p/p etc.)
    private char[] currentPattern = null;

    @Override
    public void runOpMode() throws InterruptedException {

        initHardware();   // map motors + configure directions
        initVision();     // start webcam + AprilTag processor

        telemetry.clearAll();
        telemetry.addLine("Ready. Press Play to start.");
        telemetry.update();

        // If STOP is pressed before PLAY, shut down camera safely
        if (isStopRequested()) {
            shutdownVision();
            return;
        }

        waitForStart();
        telemetry.clearAll();
        telemetry.update();

        if (isStopRequested()) {
            shutdownVision();
            return;
        }

        // Initial AprilTag scan
        mainDo();

        long start = System.currentTimeMillis();

        // -----------------------------
        // DELAY PHASE
        // Robot waits before driving forward
        // -----------------------------
        while (opModeIsActive()
                && (System.currentTimeMillis() - start) < delay) {
            mainDo();
        }

        // -----------------------------
        // DRIVE FORWARD SHORT DISTANCE
        // -----------------------------
        driveForwardFixedTimeandStop(0.4, -1);

        // -----------------------------
        // STANDSTILL PHASE
        // Keep updating AprilTag data until STOP
        // -----------------------------
        while (opModeIsActive()) {
            mainDo();
        }

        shutdownVision();
    }

    // -----------------------------
    // HARDWARE INITIALIZATION
    // -----------------------------
    private void initHardware() {

        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

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

        // Alliance tags:
        // ID 20 = blue scoring
        // ID 24 = red scoring
    }

    // -----------------------------
    // DRIVE FORWARD FOR FIXED TIME
    // -----------------------------
    private void driveForwardFixedTimeandStop(double seconds, double power) {

        setDrivePower(power);

        long start = System.currentTimeMillis();

        while (opModeIsActive()
                && (System.currentTimeMillis() - start) < (long)(seconds * 1000)) {
            mainDo();
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

    // Stop robot
    private void stopDrive() {
        setDrivePower(0.0);
    }

    // -----------------------------
    // VISION INITIALIZATION
    // -----------------------------
    private void initVision() {

        AprilTagProcessor.Builder tagBuilder = new AprilTagProcessor.Builder();
        aprilTag = tagBuilder.build();

        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        VisionPortal.Builder portalBuilder = new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(aprilTag);

        visionPortal = portalBuilder.build();
        visionPortal.resumeStreaming();
    }

    // -----------------------------
    // APRILTAG DATA UPDATE
    // -----------------------------
    private void updateAprilTagData() {

        List<AprilTagDetection> detections = aprilTag.getDetections();

        seenTagIds.clear();
        idToDistanceMeters.clear();

        for (AprilTagDetection det : detections) {

            int id = det.id;
            double distanceMeters = det.ftcPose.range;

            idToDistanceMeters.put(id, distanceMeters);

            if (!seenTagIds.contains(id)) {
                seenTagIds.add(id);
            }

            // Pattern mapping based on FTC CenterStage signal tags
            if (id == 21) {
                currentPattern = new char[]{'g', 'p', 'p'};
            } else if (id == 22) {
                currentPattern = new char[]{'p', 'g', 'p'};
            } else if (id == 23) {
                currentPattern = new char[]{'p', 'p', 'g'};
            }

            // ID 20 = blue alliance scoring
            // ID 24 = red alliance scoring
        }
    }

    // -----------------------------
    // SHUTDOWN VISION
    // -----------------------------
    private void shutdownVision() {
        if (visionPortal != null) {
            visionPortal.stopStreaming();
        }
    }

    // -----------------------------
    // MAIN LOOP TASKS
    // -----------------------------
    private void mainDo() {
        updateAprilTagData();
    }
}
