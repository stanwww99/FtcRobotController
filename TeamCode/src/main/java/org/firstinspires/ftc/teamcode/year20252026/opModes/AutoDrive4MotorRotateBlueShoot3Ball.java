package org.firstinspires.ftc.teamcode.year20252026.opModes;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.year20252026.control.*;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

@Autonomous(name="AutoBlueShoot3BallWithCamera", group="Autonomous")
public class AutoDrive4MotorRotateBlueShoot3Ball extends LinearOpMode {

    // Delay before backing up after shooting
    private int delay = 0;

    // 4‑motor drivetrain
    private DcMotor leftFront, leftBack, rightFront, rightBack;

    // Mechanisms
    private DcMotor intake;
    private Shooter shooter;
    private Carousel carousel;

    // Smart servo (pusher)
    private Servo pusher;

    // Color sensor for ball identification
    private NormalizedColorSensor colorSensor;

    // AprilTag + camera subsystem
    private Camera camera;

    // AprilTag pattern (g/p/p etc.)
    private char[] currentPattern;

    // IMU for heading control
    IMU imu;

    // AprilTag ID for blue alliance goal
    private final int BLUE_GOAL_ID = 20;

    int idx = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize motors, sensors, IMU, camera, shooter, carousel
        initHardware();

        telemetry.clearAll();
        telemetry.addLine("Ready. Press Play to start.");
        telemetry.update();

        // If STOP is pressed before PLAY, shut down camera safely
        if (isStopRequested()) {
            camera.shutdownVision();
            return;
        }

        waitForStart();
        telemetry.clearAll();
        telemetry.update();

        // Reset IMU heading to zero at start
        imu.resetYaw();

        // Begin autonomous path for BLUE alliance
        rotateToAngle(-36, 0.7);           // initial alignment
        driveForwardFixedTimeandStop(0.7, 1);
        rotateToAngle(-115, 0.5);         // face AprilTag region

        // Try to detect AprilTag pattern
        long start = System.currentTimeMillis();
        while (opModeIsActive()
                && (currentPattern = camera.getPattern()) == null
                && System.currentTimeMillis() - start < 500) {
            mainDo();
        }

        // If still no tag, perform search sweep
        if(currentPattern == null)
            searchForTag();

        mainDo();

        // Return to shooting angle
        rotateToAngle(-36, -0.5);

        // Shoot 3 balls based on detected pattern
        hardCodeShoot(2500);

        // Rotate to exit angle
        rotateToAngle(-78, 0.7);

        // Delay before backing up
        start = System.currentTimeMillis();
        while (opModeIsActive()
                && (System.currentTimeMillis() - start) < delay) {
            mainDo();
        }

        // Drive backward to parking zone
        driveForwardFixedTimeandStop(1.4, 1);

        // Standstill loop: keep updating AprilTag data until STOP
        while (opModeIsActive()) {
            if (isStopRequested()) {
                camera.shutdownVision();
                MyGyro.lastKnownHeading =
                        imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                return;
            }
        }

        // Save heading for TeleOp
        MyGyro.lastKnownHeading =
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        camera.shutdownVision();
    }

    /**
     * Performs a sweep to find the AprilTag if initial detection fails.
     */
    private void searchForTag() {
        rotateToAngle(140, -0.3);
        rotateToAngle(36, 0.5);
    }

    /**
     * Rotates robot until IMU yaw is within ±2.5° of target angle.
     */
    private void rotateToAngle(double angle, double power){
        setRotatePower(power);

        while (!(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < angle + 2.5
                && imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > angle - 2.5)) {

            if(currentPattern == null)
                setRotatePower(power);
        }

        stopDrive();
    }

    /**
     * Initializes all motors, sensors, IMU, and subsystems.
     */
    private void initHardware() {

        // Drive motors
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        // Mechanisms
        carousel = new Carousel(hardwareMap, Carousel.AUTO);
        intake   = hardwareMap.get(DcMotor.class, "intake");
        shooter  = new Shooter(hardwareMap);
        camera   = new Camera(hardwareMap);

        // Back color sensor for ball identification
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensorBack");

        // IMU initialization using MyGyro wrapper
        MyGyro.createIMU(hardwareMap);
        imu = MyGyro.imu;

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

    /**
     * Drives forward for a fixed time, then stops.
     */
    private void driveForwardFixedTimeandStop(double seconds, double power) {
        setDrivePower(power);
        long start = System.currentTimeMillis();

        while (opModeIsActive()
                && (System.currentTimeMillis() - start) < (long)(seconds * 1000)) {
            mainDo();
        }

        stopDrive();
    }

    /**
     * Sets all drive motors to the same power (forward/backward).
     */
    private void setDrivePower(double p) {
        leftFront.setPower(p);
        leftBack.setPower(p);
        rightFront.setPower(p);
        rightBack.setPower(p);
    }

    /**
     * Rotates robot until AprilTag is centered horizontally.
     */
    private void rotateUnitAprilTag(int reqID, double power) {
        setRotatePower(power);

        while (!camera.isFacingTag(reqID) && opModeIsActive()) {
            mainDo();
        }

        stopDrive();
    }

    /**
     * Rotates robot for a fixed time.
     */
    private void rotateFixedTime(double seconds, double power) {
        long start = System.currentTimeMillis();
        setRotatePower(power);

        while (opModeIsActive()
                && (System.currentTimeMillis() - start) < (long)(seconds * 1000)) {
            mainDo();
        }

        stopDrive();
    }

    /**
     * Sets motors for rotation (tank turn).
     */
    private void setRotatePower(double p){
        leftFront.setPower(p);
        rightFront.setPower(-p);
        leftBack.setPower(p);
        rightBack.setPower(-p);
    }

    private void stopDrive() {
        setDrivePower(0.0);
    }

    /**
     * Shoots 3 balls based on detected AprilTag pattern.
     * Pattern determines carousel rotation order.
     */
    private void hardCodeShoot(double RPM) {

        // Pattern case: green ball in third slot
        if (currentPattern != null && currentPattern[2] == 'g') {
            for (int i = 0; i < 3; i++) {
                carousel.rotateThirdLeft();
                while (!carousel.isFinished()) {
                    mainDo();
                }
                shoot(RPM);
            }
        }

        // Pattern case: green ball in second slot
        else if (currentPattern != null && currentPattern[1] == 'g') {
            carousel.rotateThirdRight();
            while (!carousel.isFinished()) ;

            for (int i = 0; i < 2; i++) {
                shoot(RPM);
                carousel.rotateThirdLeft();
                while (!carousel.isFinished()) {
                    mainDo();
                }
            }
            shoot(RPM);
        }

        // Pattern case: green ball in first slot
        else {
            for (int i = 0; i < 2; i++) {
                shoot(RPM);
                carousel.rotateThirdLeft();
                while (!carousel.isFinished()) {
                    mainDo();
                }
            }
            shoot(RPM);
        }
    }

    /**
     * Executes one shot:
     * - Spin shooter to RPM
     * - Wait until RPM target reached
     * - Push ball
     * - Pause
     * - Stop shooter
     */
    private void shoot(double RPM){
        shooter.start(RPM);

        while(opModeIsActive() && !shooter.isTargetMet()){
            mainDo();
        }

        shooter.push();

        // Pause to allow ball to clear
        long currMilli = System.currentTimeMillis();
        while(opModeIsActive() && System.currentTimeMillis() - currMilli < 300){
            mainDo();
        }

        shooter.stop();
    }

    /**
     * Reads HSV from back color sensor.
     */
    private float[] readColor(){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float[] hsv = new float[3];
        Color.colorToHSV(colors.toColor(), hsv);
        return hsv;
    }

    private boolean isColorGreen(){
        float[] hsv = readColor();
        double hue = hsv[0];
        return (hue >= 100 && hue <= 180);
    }

    private boolean isColorPurple(){
        float[] hsv = readColor();
        double hue = hsv[0];
        return (hue >= 200 && hue <= 245);
    }

    /**
     * Main loop tasks:
     * - Update AprilTag detections
     * - Update shooter RPM
     * - Update telemetry
     */
    private void mainDo(){
        camera.updateAprilTagData();
        shooter.updateRPM();
        updateTelemetry();
    }

    /**
     * Displays AprilTag pattern, distance to goal, and tag IDs.
     */
    private void updateTelemetry(){
        telemetry.clearAll();
        telemetry.addData("Pattern:", Arrays.toString(currentPattern));
        telemetry.addData("Distance to Goal:", camera.getDistance(BLUE_GOAL_ID));
        telemetry.addData("Goal Position", camera.getFacing(BLUE_GOAL_ID));
        camera.display(telemetry);
        telemetry.update();
    }
}
