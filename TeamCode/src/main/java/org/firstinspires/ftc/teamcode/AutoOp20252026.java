package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AutoOp20252026", group = "Autonomous")
public class AutoOp20252026 extends LinearOpMode {

    // Drive motors
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    // Other actuators (declared to match your hardware map)
    private DcMotor intakeMotor;
    private DcMotor shooterMotor1;
    private DcMotor shooterMotor2;
    private DcMotor carouselMotor; // declared as DcMotor per your hardware list
    private Servo multiModeServo;  // smart multi-mode programmed as standard servo

    @Override
    public void runOpMode() throws InterruptedException {

        // Hardware map names must match the robot configuration
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        intakeMotor    = hardwareMap.get(DcMotor.class, "intake");
        shooterMotor1  = hardwareMap.get(DcMotor.class, "shooter1");
        shooterMotor2  = hardwareMap.get(DcMotor.class, "shooter2");
        carouselMotor  = hardwareMap.get(DcMotor.class, "carouselMotor");
        multiModeServo = hardwareMap.get(Servo.class, "multiModeServo");

        // Set direction so "forward" is positive on both sides
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // Ensure motors start with zero power
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        // Initialize carousel servo to 0 degrees (convert degrees to servo position 0.0-1.0)
        setServoToDegrees(multiModeServo, 0.0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // Safety: stop if op mode is requested to stop
        if (isStopRequested()) return;

        // Move forward without turning for 1.5 seconds using only the four drive motors
        double forwardPower = -0.5; // reasonable default; adjust if needed
        leftFront.setPower(forwardPower);
        leftBack.setPower(forwardPower);
        rightFront.setPower(forwardPower);
        rightBack.setPower(forwardPower);

        sleep(1500); // 1500 milliseconds = 1.5 seconds

        // Stop and hold standstill
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        telemetry.addData("Auto", "Completed move forward 1.5s and stopped");
        telemetry.update();

        // keep op mode alive until stop is pressed, or simply return to end
        // sleep(1000);
    }

    /**
     * Convert degrees (0-180 typical) to servo position (0.0 - 1.0) and set it.
     * If your smart servo uses a different range, adjust the conversion accordingly.
     */
    private void setServoToDegrees(Servo servo, double degrees) {
        // clamp degrees between 0 and 180
        if (degrees < 0) degrees = 0;
        if (degrees > 180) degrees = 180;
        double position = degrees / 180.0;
        servo.setPosition(position);
    }
}
