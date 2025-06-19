package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Joystick Chassis Drive 2 Motor", group = "TeleOp")
public class ChassisJoystickTwoMotor extends LinearOpMode {

    // Declare motors
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;

    // Variables for tracking speed and acceleration
    private double previousSpeed = 0.0;
    private double previousTime  = 0.0;

    @Override
    public void runOpMode() {

        // Initialize hardware variables. Ensure these names match your robot configuration.
        leftMotor  = hardwareMap.get(DcMotor.class, "left_motor");
        rightMotor = hardwareMap.get(DcMotor.class, "right_motor");

        // Reverse one motor so that the robot moves forward when both motors are powered equally.
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the start button.
        waitForStart();
        previousTime = getRuntime();

        while (opModeIsActive()) {

            // Read joystick inputs:
            // - left_stick_y controls forward/backward motion (inverted for intuitive control)
            // - left_stick_x controls turning
            double drive = -gamepad1.left_stick_y; // Negative because pushing forward gives a negative value.
            double turn  = gamepad1.left_stick_x;

            // Calculate motor power values.
            double leftPower  = Range.clip(drive + turn, -1.0, 1.0);
            double rightPower = Range.clip(drive - turn, -1.0, 1.0);

            // Command the motors.
            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);

            // Compute the robot speed as the average of left and right motor power.
            double currentSpeed = (leftPower + rightPower) / 2.0;
            double currentTime  = getRuntime();
            double deltaTime    = currentTime - previousTime;
            double acceleration = 0.0;

            if (deltaTime > 0) {
                acceleration = (currentSpeed - previousSpeed) / deltaTime;
            }

            // Update telemetry with the calculated values.
            telemetry.addData("Elapsed Time", currentTime);
            telemetry.addData("Robot Speed", currentSpeed);
            telemetry.addData("Robot Acceleration", acceleration);
            telemetry.addData("Motor Left Power", leftPower);
            telemetry.addData("Motor Right Power", rightPower);
            telemetry.update();

            // Update the previous values for the next iteration.
            previousSpeed = currentSpeed;
            previousTime  = currentTime;
        }
    }
}
