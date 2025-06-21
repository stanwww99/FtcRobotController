package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Chassis TeleOp 4 motor", group = "Linear OpMode")
public class ChassisJoystickFourMotor extends LinearOpMode {

    // Declare motor objects
    private DcMotor leftMotor1;
    private DcMotor leftMotor2;
    private DcMotor rightMotor1;
    private DcMotor rightMotor2;

    // Encoder to distance conversion constants
    static final double TICKS_PER_REV = 1120.0;         // Adjust as needed for your motor encoder
    static final double WHEEL_DIAMETER = 0.1;             // in meters (e.g., 0.1 m = 10 cm)
    static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;

    @Override
    public void runOpMode() throws InterruptedException {
        // Map hardware using names from the configuration on the Robot Controller phone
        leftMotor1 = hardwareMap.get(DcMotor.class, "leftMotor1");
        leftMotor2 = hardwareMap.get(DcMotor.class, "leftMotor2");
        rightMotor1 = hardwareMap.get(DcMotor.class, "rightMotor1");
        rightMotor2 = hardwareMap.get(DcMotor.class, "rightMotor2");

        // Set proper motor directions (assuming the right side motors need to be reversed)
        leftMotor1.setDirection(DcMotor.Direction.FORWARD);
        leftMotor2.setDirection(DcMotor.Direction.FORWARD);
        rightMotor1.setDirection(DcMotor.Direction.REVERSE);
        rightMotor2.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders
        leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        // Set them to run using encoders for speed calculations
        leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Create an elapsed time object to track runtime
        ElapsedTime runtime = new ElapsedTime();
        double previousTime = runtime.seconds();
        double previousSpeed = 0.0; // in m/s

        // Record initial encoder positions (assuming leftMotor1 and rightMotor1 represent each side)
        int lastLeftEncoder = leftMotor1.getCurrentPosition();
        int lastRightEncoder = rightMotor1.getCurrentPosition();

        telemetry.clearAll();
        telemetry.update();

        waitForStart(); // Wait for start command from Driver Station

        // Main loop that runs while the op mode is active
        while (opModeIsActive()) {
            // =========================
            // 1. Read Joystick Input
            // =========================
            // Forward/backward: gamepad1.left_stick_y (inverted so pushing forward gives positive value)
            // Turning: gamepad1.left_stick_x
            double forward = -gamepad1.left_stick_y;
            double turn = gamepad1.left_stick_x;

            // Mix the joystick values to determine motor power for tank drive
            double leftPower = forward + turn;
            double rightPower = forward - turn;

            // Ensure the power values are within [-1, 1]
            leftPower = Range.clip(leftPower, -1.0, 1.0);
            rightPower = Range.clip(rightPower, -1.0, 1.0);

            // ==========================
            // 2. Set Motor Powers
            // ==========================
            leftMotor1.setPower(leftPower);
            leftMotor2.setPower(leftPower);
            rightMotor1.setPower(rightPower);
            rightMotor2.setPower(rightPower);

            // ================================
            // 3. Compute Telemetry Data
            // ================================
            double currentTime = runtime.seconds();
            double dt = currentTime - previousTime;
            previousTime = currentTime;

            // Get current encoder positions from one motor per side
            int currentLeftEncoder = leftMotor1.getCurrentPosition();
            int currentRightEncoder = rightMotor1.getCurrentPosition();

            // Calculate the change in encoder ticks since the last loop
            int deltaLeft = currentLeftEncoder - lastLeftEncoder;
            int deltaRight = currentRightEncoder - lastRightEncoder;
            lastLeftEncoder = currentLeftEncoder;
            lastRightEncoder = currentRightEncoder;

            // Convert encoder ticks to distance traveled (in meters)
            double distanceLeft = (deltaLeft / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE;
            double distanceRight = (deltaRight / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE;

            // Compute an average distance (if driving straight both sides should be similar)
            double avgDistance = (distanceLeft + distanceRight) / 2.0;

            // Estimate speed (m/s) as distance / time interval
            double speed = dt > 0 ? avgDistance / dt : 0;

            // Estimate acceleration (m/s²) as change in speed / time interval
            double acceleration = dt > 0 ? (speed - previousSpeed) / dt : 0;
            previousSpeed = speed;

            // =========================================
            // 4. Telemetry Output (Driver Station + REV Log)
            // =========================================
            telemetry.addData("Elapsed Time (s)", String.format("%.2f", currentTime));
            telemetry.addData("Speed (m/s)", String.format("%.2f", speed));
            telemetry.addData("Acceleration (m/s²)", String.format("%.2f", acceleration));
            telemetry.addData("Left Motor Power", String.format("%.2f", leftPower));
            telemetry.addData("Right Motor Power", String.format("%.2f", rightPower));
            telemetry.update();

            // Allow hardware to catch up and prevent CPU overuse
            idle();
        }
    }
}