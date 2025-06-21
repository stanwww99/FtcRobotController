package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Chassis Drive TeleOp 4 Motor", group = "TeleOp")
public class ChassisJoystickFourMotor extends OpMode {

    // Declare the left and right motors (two on each side)
    private DcMotor leftMotor1;
    private DcMotor leftMotor2;
    private DcMotor rightMotor1;
    private DcMotor rightMotor2;

    // ElapsedTime is used for calculating elapsed time and dt
    private ElapsedTime runtime = new ElapsedTime();

    // Variables to compute speed and acceleration over time
    private double previousSpeed = 0.0;
    private double previousTime = 0.0;

    @Override
    public void init() {
        // Map the motors from the hardware configuration
        leftMotor1 = hardwareMap.get(DcMotor.class, "leftMotor1");
        leftMotor2 = hardwareMap.get(DcMotor.class, "leftMotor2");
        rightMotor1 = hardwareMap.get(DcMotor.class, "rightMotor1");
        rightMotor2 = hardwareMap.get(DcMotor.class, "rightMotor2");

        // It is common for the right motors to be reversed so that both sides move
        // in the correct direction.
        rightMotor1.setDirection(DcMotor.Direction.REVERSE);
        rightMotor2.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        // Reset the runtime when the OpMode starts
        runtime.reset();
        previousTime = 0.0;
        previousSpeed = 0.0;
    }

    @Override
    public void loop() {
        // Read the joystick's Y and X values.
        // Use gamepad1.left_stick_y for forward/backward (invert it so that pushing forward is positive)
        // and gamepad1.left_stick_x for turning.
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;

        // Calculate motor powers for differential (arcade) drive.
        // Adding x to the forward component (y) for left, and subtracting x for right.
        double leftPower = y + x;
        double rightPower = y - x;

        // Normalize motor powers so that values stay in [-1, 1]
        double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (maxPower > 1.0) {
            leftPower /= maxPower;
            rightPower /= maxPower;
        }

        // Set the motor powers. Both left motors receive the same value,
        // and both right motors receive the same value.
        leftMotor1.setPower(leftPower);
        leftMotor2.setPower(leftPower);
        rightMotor1.setPower(rightPower);
        rightMotor2.setPower(rightPower);

        // Compute a simple "robot speed" as the average of the absolute motor powers.
        double currentSpeed = (Math.abs(leftPower) + Math.abs(rightPower)) / 2.0;

        // Calculate elapsed time and time difference (dt) since the last loop cycle.
        double currentTime = runtime.seconds();
        double dt = currentTime - previousTime;

        // Calculate acceleration (change in speed per second).
        double acceleration = 0.0;
        if (dt > 0) {
            acceleration = (currentSpeed - previousSpeed) / dt;
        }

        // Update previous time and speed for the next cycle.
        previousSpeed = currentSpeed;
        previousTime = currentTime;

        // Use telemetry to send data back to the driver station:
        telemetry.addData("Elapsed Time", String.format("%.2f sec", currentTime));
        telemetry.addData("Robot Speed", String.format("%.2f", currentSpeed));
        telemetry.addData("Robot Acceleration", String.format("%.2f", acceleration));
        telemetry.addData("Left Motor Power", String.format("%.2f", leftPower));
        telemetry.addData("Right Motor Power", String.format("%.2f", rightPower));
        telemetry.update();
    }
}