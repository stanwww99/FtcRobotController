package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "ManualDrive4Motor_WithLowSpeed", group = "TeleOp")
public class ManualDrive4Motor_WithLowSpeed extends LinearOpMode {

    // Drive motors
    private DcMotor leftFront, leftRear, rightFront, rightRear;

    // Low speed limit
    private static final double NORMAL_MAX = 1.0;
    private static final double LOW_SPEED_MAX = 0.3;

    @Override
    public void runOpMode() {
        // Hardware mapping - change names to match your robot config
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear   = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear  = hardwareMap.get(DcMotor.class, "rightBack");

        // Set motor directions depending on physical wiring
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        // Use brake behavior for more predictable stopping (optional)
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            // Read driver gamepad (gamepad1)
            Gamepad g = gamepad1;

            // Left joystick: translation
            // Note: In FTC, forward on stick is negative (up = -y), so invert y to make up = positive forward
            double leftStickX = applyDeadzone(g.left_stick_x);
            double leftStickY = -applyDeadzone(g.left_stick_y);

            // Right joystick: rotation (spin)
            double rightStickX = applyDeadzone(g.right_stick_x);

            // Low speed boolean (RT touched/held). Use trigger threshold > 0.05
            boolean lowSpeedMode = g.right_trigger > 0.05;

            double maxAllowed = lowSpeedMode ? LOW_SPEED_MAX : NORMAL_MAX;

            // Compute translation components
            // For mecanum-like strafing mapping on a 4-motor tank-chassis layout that supports strafing via hardware:
            // - leftStickY controls forward/back
            // - leftStickX controls strafing left/right
            // If your drivetrain cannot physically strafe, remove leftStickX contribution and map to turning or ignore.
            double forward = leftStickY;                // forward/backward
            double strafe  = leftStickX;                // left/right strafing
            double spin    = rightStickX;               // rotation (positive = clockwise)

            // Scale spin separately by the right stick magnitude (already represented by rightStickX)
            // Compose motor powers: translation + rotation mixing
            double lf = forward + strafe + spin;
            double lr = forward - strafe + spin;
            double rf = forward - strafe - spin;
            double rr = forward + strafe - spin;

            // Find the largest magnitude among the computed powers
            double maxMagnitude = Math.max(
                    Math.max(Math.abs(lf), Math.abs(lr)),
                    Math.max(Math.abs(rf), Math.abs(rr))
            );

            // Normalize if any power is outside [-1,1]
            if (maxMagnitude > 1.0) {
                lf /= maxMagnitude;
                lr /= maxMagnitude;
                rf /= maxMagnitude;
                rr /= maxMagnitude;
            }

            // Apply the allowed max (low speed or normal)
            lf = clamp(lf, -maxAllowed, maxAllowed);
            lr = clamp(lr, -maxAllowed, maxAllowed);
            rf = clamp(rf, -maxAllowed, maxAllowed);
            rr = clamp(rr, -maxAllowed, maxAllowed);

            // Set motor powers
            leftFront.setPower(lf);
            leftRear.setPower(lr);
            rightFront.setPower(rf);
            rightRear.setPower(rr);

            // Telemetry for debugging and validation
            telemetry.addData("LF", "%.2f", lf);
            telemetry.addData("LR", "%.2f", lr);
            telemetry.addData("RF", "%.2f", rf);
            telemetry.addData("RR", "%.2f", rr);
            telemetry.addData("LeftStickX,Y", "%.2f, %.2f", leftStickX, leftStickY);
            telemetry.addData("RightStickX (spin)", "%.2f", rightStickX);
            telemetry.addData("LowSpeedMode", lowSpeedMode);
            telemetry.addData("MaxAllowed", "%.2f", maxAllowed);
            telemetry.update();
        }
    }

    // Utility: simple deadzone for joystick jitter
    private double applyDeadzone(double value) {
        double dz = 0.05;
        if (Math.abs(value) < dz) return 0.0;
        // Optional: scale remaining range to smooth control (linear here)
        return value;
    }

    private double clamp(double v, double min, double max) {
        return Range.clip(v, min, max);
    }
}
