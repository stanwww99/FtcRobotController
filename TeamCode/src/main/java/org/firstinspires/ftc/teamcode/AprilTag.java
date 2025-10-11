package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "AprilTag ID Reader", group = "Autonomous")
public class AprilTag extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() {
        // Create AprilTag processor
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create VisionPortal with the AprilTag processor
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        char[] pattern = new char[3]; // Will hold g/p values

        telemetry.addLine("Waiting for start...");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Get all current detections
            List<AprilTagDetection> detections = aprilTag.getDetections();

            for (AprilTagDetection detection : detections) {
                int id = detection.id;
                if (id == 21) {
                    pattern = new char[]{'g', 'p', 'p'};
                } else if (id == 22) {
                    pattern = new char[]{'p', 'g', 'p'};
                } else if (id == 23) {
                    pattern = new char[]{'p', 'p', 'g'};
                }

                telemetry.addData("Seen ID", id);
                telemetry.addData("Pattern", new String(pattern));
            }

            telemetry.update();
        }

        // Stop vision when done
        visionPortal.close();
    }
}
