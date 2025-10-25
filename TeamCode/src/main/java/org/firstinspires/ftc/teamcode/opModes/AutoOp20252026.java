package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

@Autonomous(name="AutoOp20252026", group="Autonomous")
public class AutoOp20252026 extends LinearOpMode {

    // Drive motors
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    // Mechanism motors
    private DcMotor intake, shooter, carousel;
    // Smart servo
    private Servo smartServo;

    // Vision
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // Data structures
    private final HashMap<Integer, Double> idToDistanceMeters = new HashMap<>();
    private final List<Integer> seenTagIds = new ArrayList<>();
    private char[] currentPattern = null;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        initVision();

        waitForStart();
        if (isStopRequested()) {
            shutdownVision();
            return;
        }

        // Drive forward for 1.5s
        driveForwardFixedTime(1.5, -0.8);
        stopDrive();

        // Standstill, keep updating AprilTag data
        while (opModeIsActive() && !isStopRequested()) {
            updateAprilTagData();
            stopDrive();
            sleep(20);
        }

        shutdownVision();
    }

    private void initHardware() {
        // Drive motors
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{leftFront, leftBack, rightFront, rightBack}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Other motors
        intake   = hardwareMap.get(DcMotor.class, "intake");
        shooter  = hardwareMap.get(DcMotor.class, "shooter");
        carousel = hardwareMap.get(DcMotor.class, "carousel");

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carousel.setPower(0.0); // initialize carousel motor to 0

        // Smart servo (Standard Mode ±150°)
        smartServo = hardwareMap.get(Servo.class, "smartServo");
        smartServo.setPosition(0.0);

        // Shooter motor reference: goBILDA 5202 1:1, 6000 rpm
        // Carousel motor reference: goBILDA 5202 99.5:1, ~60 rpm
        // Servo reference: Studica Multi-Mode Smart Servo (Standard Mode)
        // Alliance tags: ID 20 (blue scoring), ID 24 (red scoring)
    }

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

    private void driveForwardFixedTime(double seconds, double power) {
        long start = System.currentTimeMillis();
        setDrivePower(power);
        while (opModeIsActive() && !isStopRequested()
                && (System.currentTimeMillis() - start) < (long)(seconds * 1000)) {
            updateAprilTagData();
            sleep(10);
        }
        stopDrive();
    }

    private void setDrivePower(double p) {
        leftFront.setPower(p);
        leftBack.setPower(p);
        rightFront.setPower(p);
        rightBack.setPower(p);
    }

    private void stopDrive() {
        setDrivePower(0.0);
    }

    private void updateAprilTagData() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        seenTagIds.clear();

        for (AprilTagDetection det : detections) {
            int id = det.id;
            double distanceMeters = det.ftcPose.range;
            idToDistanceMeters.put(id, distanceMeters);
            if (!seenTagIds.contains(id)) {
                seenTagIds.add(id);
            }

            if (id == 21) {
                currentPattern = new char[]{'g', 'p', 'p'};
            } else if (id == 22) {
                currentPattern = new char[]{'p', 'g', 'p'};
            } else if (id == 23) {
                currentPattern = new char[]{'p', 'p', 'g'};
            }
            // ID 20 = blue alliance scoring, ID 24 = red alliance scoring
        }
    }

    private void shutdownVision() {
        if (visionPortal != null) {
            visionPortal.stopStreaming();
        }
    }
}