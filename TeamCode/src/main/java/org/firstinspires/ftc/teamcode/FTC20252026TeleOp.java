package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "FTC20252026TeleOp", group = "TeleOp")
public class FTC20252026TeleOp extends LinearOpMode {

    // Drive motors
    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor rightFront;
    private DcMotor rightRear;

    // Intake motor
    private DcMotor intake;
    //Shooter motor
    private DcMotor shooterleft;
    private DcMotor shooterright;

    // Mode flags
    private boolean lowSpeedMode = false;      // true when right trigger on gamepad1 is pressed
    private boolean intakeOn = false;          // toggled by gamepad2.x

    // For button-edge detection
    private boolean previousGamepad2X = false;

    // For speed/acceleration telemetry
    private double previousSpeed = 0.0;
    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        // Hardware map - change names to match your robot configuration
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRear");

        intake = hardwareMap.get(DcMotor.class, "intake");

        // Set motor directions if needed (adjust if your motors are reversed)
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        // Ensure zero power behavior if desired
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        loopTimer.reset();
        previousSpeed = 0.0;
        previousGamepad2X = false;

        while (opModeIsActive()) {

            double loopDt = Math.max(1e-6, loopTimer.seconds());
            loopTimer.reset();

            // --- Driving input from gamepad1 ---
            // Right stick Y controls forward/back (stick up is -1, so invert)
            double forward = -gamepad1.right_stick_y;       // forward positive
            // Left stick X controls turning left/right
            double turn = gamepad1.left_stick_x;           // right positive

            // Compute raw motor powers
            double leftPowerRaw  = forward + turn;
            double rightPowerRaw = forward - turn;

            // Normalize powers if any > 1
            double max = Math.max(Math.abs(leftPowerRaw), Math.abs(rightPowerRaw));
            if (max > 1.0) {
                leftPowerRaw  /= max;
                rightPowerRaw /= max;
            }

            // Low speed mode when right trigger touched / held on gamepad1
            lowSpeedMode = gamepad1.right_trigger > 0.05;   // threshold to avoid accidental touch

            double speedMultiplier = lowSpeedMode ? 0.30 : 1.0;

            double leftPower  = leftPowerRaw  * speedMultiplier;
            double rightPower = rightPowerRaw * speedMultiplier;

            // Apply motor powers
            leftFront.setPower(leftPower);
            leftRear.setPower(leftPower);
            rightFront.setPower(rightPower);
            rightRear.setPower(rightPower);

            // --- Intake control (gamepad2) ---
            // X toggles intake on/off (edge detect)
            boolean currentGamepad2X = gamepad2.x;
            if (currentGamepad2X && !previousGamepad2X) {
                // button pressed now, toggle
                intakeOn = !intakeOn;
            }
            previousGamepad2X = currentGamepad2X;

            double intakePower = 0.0;

            // LB: intake motor go backward at full speed to spit out
            if (gamepad2.left_bumper) {
                intakePower = -1.0;    // negative = spit out per your request
            }
            // LT: make intake motor go forward to launch, analog scaling by trigger
            else if (gamepad2.left_trigger > 0.05) {
                // left_trigger is 0.0 .. 1.0, use that as power for forward launch
                intakePower = RangeClip(gamepad2.left_trigger, 0.0, 1.0);
            }
            // X toggle: if intakeOn true, run intake forward at full speed (or adjust as desired)
            else if (intakeOn) {
                intakePower = 1.0;
            } else {
                intakePower = 0.0;
            }

            intake.setPower(intakePower);

            // --- Telemetry calculations: speed and acceleration ---
            // Define "speed" as the average of absolute left and right drive power (0..1)
            double currentSpeed = (Math.abs(leftPower) + Math.abs(rightPower)) / 2.0;
            double acceleration = (currentSpeed - previousSpeed) / loopDt; // units: (power units) per second
            previousSpeed = currentSpeed;

            // --- Telemetry ---
            telemetry.addData("Left Power", "%.3f", leftPower);
            telemetry.addData("Right Power", "%.3f", rightPower);
            telemetry.addData("Low Speed Mode", lowSpeedMode ? "TRUE (30%% max)" : "FALSE (100%%)");
            telemetry.addData("Speed (avg mag)", "%.3f", currentSpeed);
            telemetry.addData("Acceleration (power/sec)", "%.3f", acceleration);
            telemetry.addData("Intake Power", "%.3f", intakePower);
            telemetry.addData("Intake On (toggle X)", intakeOn ? "TRUE" : "FALSE");
            telemetry.update();

            // Small sleep to avoid busy-loop pegging CPU
            idle();
        }
    }

    /**
     * Clip helper (simple). Ideally use Range.clip from SDK, but implementing local to avoid imports.
     */
    private double RangeClip(double val, double min, double max) {
        if (val < min) return min;
        if (val > max) return max;
        return val;
    }
}
