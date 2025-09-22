package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoPosSweepStopGuaranteed", group = "Tests")
public class ServoPosSweepStopGuaranteed extends LinearOpMode {

    private static final String SERVO_NAME = "smart_servo"; // change to match your config
    private static final double MIN_DEG = 0.0;
    private static final double MAX_DEG = 180.0;
    private static final double STEP_DEG = 1.0;               // degrees per small step
    private static final double MOVE_SPEED_DEG_PER_SEC = 60.0; // simulated moving speed (deg/s)
    private static final long STOP_MS = 5000L;                // hold time while stopped

    private Servo servo;
    private double positionDeg = 0.0;            // commanded position in degrees
    private double reportedSpeedDegPerSec = 0.0; // simulated speed for telemetry

    @Override
    public void runOpMode() {
        servo = hardwareMap.get(Servo.class, SERVO_NAME);

        // initialize to 0 degrees at start
        positionDeg = 0.0;
        applyServoDegrees(positionDeg);
        reportedSpeedDegPerSec = 0.0;

        telemetry.addData("Status", "Init at 0 deg");
        telemetry.addData("Position (deg)", "%.1f", positionDeg);
        telemetry.addData("Speed (deg/s)", "%.1f", reportedSpeedDegPerSec);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // rotate 60 degrees right (increase)
            double targetRight = clampDeg(positionDeg + 60.0);
            moveToTarget(targetRight, MOVE_SPEED_DEG_PER_SEC);

            // Guarantee stop: set reported speed to 0 and re-apply final position (no motion command)
            reportedSpeedDegPerSec = 0.0;
            applyServoDegrees(positionDeg);
            telemetry.addData("Action", "Stopped");
            telemetry.addData("Position (deg)", "%.1f", positionDeg);
            telemetry.addData("Speed (deg/s)", "%.1f", reportedSpeedDegPerSec);
            telemetry.update();
            sleep(STOP_MS);

            // rotate 60 degrees left (decrease)
            double targetLeft = clampDeg(positionDeg - 60.0);
            moveToTarget(targetLeft, MOVE_SPEED_DEG_PER_SEC);

            // Guarantee stop again
            reportedSpeedDegPerSec = 0.0;
            applyServoDegrees(positionDeg);
            telemetry.addData("Action", "Stopped");
            telemetry.addData("Position (deg)", "%.1f", positionDeg);
            telemetry.addData("Speed (deg/s)", "%.1f", reportedSpeedDegPerSec);
            telemetry.update();
            sleep(STOP_MS);
        }
    }

    // Move from current positionDeg to targetDeg at approx speedDegPerSec (simulated).
    // Uses small discrete steps to create smooth motion and updates telemetry while moving.
    private void moveToTarget(double targetDeg, double speedDegPerSec) {
        if (Math.abs(targetDeg - positionDeg) < 1e-6) return;

        int dir = targetDeg > positionDeg ? 1 : -1;
        reportedSpeedDegPerSec = Math.abs(speedDegPerSec);

        long stepDelayMs = (long) Math.max(1, Math.round((STEP_DEG / reportedSpeedDegPerSec) * 1000.0));

        while (opModeIsActive() && ((dir > 0 && positionDeg < targetDeg) || (dir < 0 && positionDeg > targetDeg))) {
            double next = positionDeg + dir * STEP_DEG;
            if ((dir > 0 && next > targetDeg) || (dir < 0 && next < targetDeg)) next = targetDeg;
            positionDeg = clampDeg(next);
            applyServoDegrees(positionDeg);

            telemetry.addData("Action", "Moving");
            telemetry.addData("Target (deg)", "%.1f", targetDeg);
            telemetry.addData("Position (deg)", "%.1f", positionDeg);
            telemetry.addData("Speed (deg/s)", "%.1f", reportedSpeedDegPerSec);
            telemetry.update();

            try {
                Thread.sleep(stepDelayMs);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }

        // After reaching target, immediately set reported speed to 0 to guarantee stop state
        reportedSpeedDegPerSec = 0.0;
    }

    // Map degrees to servo position 0.0..1.0 and apply to hardware
    private void applyServoDegrees(double deg) {
        double clamped = clampDeg(deg);
        double normalized = (clamped - MIN_DEG) / (MAX_DEG - MIN_DEG);
        normalized = Math.max(0.0, Math.min(1.0, normalized));
        servo.setPosition(normalized);
    }

    private double clampDeg(double d) {
        if (d < MIN_DEG) return MIN_DEG;
        if (d > MAX_DEG) return MAX_DEG;
        return d;
    }
}
