package org.firstinspires.ftc.teamcode.year20252026.testOpModes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

@Autonomous(name="CameraTest", group="Autonomous")
public class CameraTest extends LinearOpMode {

    // VisionPortal manages camera streaming + image processing pipeline
    private VisionPortal visionPortal;

    // AprilTagProcessor performs tag detection + pose estimation
    private AprilTagProcessor aprilTag;

    // Maps AprilTag ID → distance (meters) from robot
    private final Map<Integer, Double> idToDistanceMeters = new HashMap<>();

    // List of tag IDs seen in the current frame
    private final List<Integer> seenTagIds = new ArrayList<>();

    // A 3‑character pattern determined by tag ID (custom game logic)
    private char[] currentPattern = new char[3];

    @Override
    public void runOpMode() throws InterruptedException {

        // Initial telemetry before starting
        telemetry.clearAll();
        telemetry.addLine("Ready. Press Play to start.");
        telemetry.update();

        // Initialize camera + AprilTag pipeline
        initVision();

        // If STOP is pressed before PLAY, shut down cleanly
        if (isStopRequested()) {
            shutdownVision();
            return;
        }

        // Wait for PLAY
        waitForStart();

        // Main loop: continuously read AprilTag detections
        while (opModeIsActive()) {

            updateAprilTagData();

            // Display processed tag information
            telemetry.clearAll();
            telemetry.addData("Seen AprilTags", seenTagIds);
            telemetry.addData("Distance to AprilTags", idToDistanceMeters);
            telemetry.addData("Pattern", Arrays.toString(currentPattern));
            telemetry.update();
        }

        // Stop camera streaming when OpMode ends
        shutdownVision();
    }

    /**
     * Initializes the VisionPortal and AprilTag processor.
     * Sets up webcam input → AprilTag detection pipeline.
     */
    private void initVision() {

        // Build AprilTag processor with default settings
        AprilTagProcessor.Builder tagBuilder = new AprilTagProcessor.Builder();
        aprilTag = tagBuilder.build();

        // Retrieve webcam from Robot Configuration
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        // Build VisionPortal:
        // - Assign webcam
        // - Attach AprilTag processor
        VisionPortal.Builder portalBuilder = new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(aprilTag);

        // Create portal and begin streaming frames to processors
        visionPortal = portalBuilder.build();
        visionPortal.resumeStreaming();
    }

    /**
     * Reads AprilTag detections from the processor,
     * extracts ID + distance, and updates custom pattern logic.
     */
    private void updateAprilTagData() {

        // Get list of all tags detected in the current frame
        List<AprilTagDetection> detections = aprilTag.getDetections();

        // Clear previous frame data
        seenTagIds.clear();
        idToDistanceMeters.clear();

        // Process each detection
        for (AprilTagDetection det : detections) {

            int id = det.id;

            // Distance from robot to tag (meters)
            double distanceMeters = det.ftcPose.range;

            // Store distance in map
            idToDistanceMeters.put(id, distanceMeters);

            // Track unique tag IDs seen this frame
            if (!seenTagIds.contains(id)) {
                seenTagIds.add(id);
            }

            // --- CUSTOM GAME LOGIC ---
            // Assign a 3‑character pattern based on tag ID.
            // These IDs correspond to FTC CenterStage signal tags.
            if (id == 21) {
                currentPattern = new char[]{'g', 'p', 'p'};
            } else if (id == 22) {
                currentPattern = new char[]{'p', 'g', 'p'};
            } else if (id == 23) {
                currentPattern = new char[]{'p', 'p', 'g'};
            }

            // Additional tag meaning:
            // ID 20 = blue alliance scoring
            // ID 24 = red alliance scoring
        }
    }

    /**
     * Stops camera streaming and releases resources.
     */
    private void shutdownVision() {
        if (visionPortal != null) {
            visionPortal.stopStreaming();
        }
    }
}
