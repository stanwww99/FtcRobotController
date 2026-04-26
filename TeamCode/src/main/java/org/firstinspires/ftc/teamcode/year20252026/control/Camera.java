package org.firstinspires.ftc.teamcode.year20252026.control;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Camera {

    // How close the tag's X‑offset must be to "center" for the robot to be considered facing it.
    // det.ftcPose.x ≈ horizontal displacement of tag relative to camera.
    int cameraPositionTolerance = 5;

    // VisionPortal manages camera streaming + image processing pipeline.
    private VisionPortal visionPortal;

    // AprilTagProcessor performs tag detection + pose estimation.
    private AprilTagProcessor aprilTag;

    // Maps AprilTag ID → distance (meters) from robot.
    private final Map<Integer, Double> idToDistanceMeters = new HashMap<>();

    // List of tag IDs seen in the current frame.
    private final List<Integer> seenTagIds = new ArrayList<>();

    // Raw AprilTag detections from the processor.
    private List<AprilTagDetection> detections;

    /**
     * Constructor: sets up webcam + AprilTag pipeline.
     * This class is meant to be used inside OpModes as a reusable vision module.
     */
    public Camera(HardwareMap hardwareMap){

        // Build AprilTag processor with default settings.
        AprilTagProcessor.Builder tagBuilder = new AprilTagProcessor.Builder();
        aprilTag = tagBuilder.build();

        // Retrieve webcam from Robot Configuration.
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        // Build VisionPortal:
        // - Assign webcam
        // - Attach AprilTag processor
        VisionPortal.Builder portalBuilder = new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(aprilTag);

        // Create portal and begin streaming frames to processors.
        visionPortal = portalBuilder.build();
        visionPortal.resumeStreaming();

        // Initialize detection list.
        detections = new ArrayList<>();
    }

    /**
     * Checks whether the robot is facing a specific AprilTag.
     * "Facing" means the tag's horizontal offset (ftcPose.x) is near zero.
     *
     * @param reqID the tag ID we want to check
     * @return true if the tag is centered within tolerance
     */
    public boolean isFacingTag(int reqID){
        for (AprilTagDetection det : detections) {

            // ftcPose.x = horizontal displacement of tag relative to camera center.
            double position = det.ftcPose.x;

            // If the tag ID matches and its X offset is small, robot is facing it.
            if(det.id == reqID &&
                    position > -cameraPositionTolerance &&
                    position <  cameraPositionTolerance) {

                return true;
            }
        }
        return false;
    }

    /**
     * Returns the horizontal offset (ftcPose.x) of a specific tag.
     * Useful for alignment logic (e.g., PID turning to center the tag).
     *
     * @param reqID the tag ID we want
     * @return X offset, or -1 if tag not found
     */
    public double getFacing(int reqID){
        for (AprilTagDetection det : detections) {
            if(det.id == reqID)
                return det.ftcPose.x;
        }
        return -1;
    }

    /**
     * Returns the distance (meters) to a specific AprilTag.
     * det.ftcPose.range is computed by the AprilTag library using tag size + camera intrinsics.
     *
     * @param reqID the tag ID we want
     * @return distance in meters, or -1 if tag not found
     */
    public double getDistance(int reqID){
        for (AprilTagDetection det : detections) {
            if(det.id == reqID)
                return det.ftcPose.range;
        }
        return -1;
    }

    /**
     * Determines the obelisk pattern based on AprilTag ID.
     * These IDs correspond to FTC CenterStage signal tags.
     *
     * @return a char[] representing the pattern, or null if no pattern tag is seen
     */
    public char[] getPattern() {

        int id = 0;

        for (AprilTagDetection det : detections) {
            id = det.id;

            // Pattern mapping logic:
            if (id == 21) {
                return new char[]{'g', 'p', 'p'};
            } else if (id == 22) {
                return new char[]{'p', 'g', 'p'};
            } else if (id == 23) {
                return new char[]{'p', 'p', 'g'};
            }

            // Additional tag meaning:
            // ID 20 = blue alliance scoring
            // ID 24 = red alliance scoring
        }

        return null;
    }

    /**
     * Updates internal detection list and distance map.
     * Must be called every loop by the OpMode using this Camera class.
     */
    public void updateAprilTagData() {

        // Pull latest detections from processor.
        detections = aprilTag.getDetections();

        // Clear previous frame data.
        seenTagIds.clear();
        idToDistanceMeters.clear();

        // Process each detection.
        for (AprilTagDetection det : detections) {

            int id = det.id;
            double distanceMeters = det.ftcPose.range;

            // Store distance.
            idToDistanceMeters.put(id, distanceMeters);

            // Track unique tag IDs.
            if (!seenTagIds.contains(id)) {
                seenTagIds.add(id);
            }
        }
    }

    /**
     * Returns list of tag IDs seen in the current frame.
     */
    public List<Integer> getSeenTagIds(){
        return seenTagIds;
    }

    /**
     * Returns map of tag ID → distance (meters).
     */
    public Map<Integer, Double> getIdsToDistance(){
        return idToDistanceMeters;
    }

    /**
     * Stops camera streaming and releases resources.
     * Should be called when OpMode ends.
     */
    public void shutdownVision() {
        if (visionPortal != null) {
            visionPortal.stopStreaming();
        }
    }

    /**
     * Displays tag IDs to telemetry.
     * Useful for debugging from OpModes that use this Camera class.
     */
    public void display(Telemetry telemetry){
        for (AprilTagDetection det : detections) {
            telemetry.addData("ID: ", det.id);
        }
    }
}
