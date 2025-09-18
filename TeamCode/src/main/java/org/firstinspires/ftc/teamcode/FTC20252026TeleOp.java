package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="FTC20252026TeleOp", group="TeleOp")
public class FTC20252026TeleOp extends LinearOpMode {

    // Drive motors
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    // Intake motor
    private DcMotor intake;
    // Shooter motors (to be configured later)
    private DcMotor shooterLeft, shooterRight;

    // Low‐speed mode flag
    private boolean lowSpeedMode = false;

    @Override
    public void runOpMode() {
        // Map hardware
        leftFront  = hardwareMap.get(DcMotor.class, "left_front");
        leftBack   = hardwareMap.get(DcMotor.class, "left_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightBack  = hardwareMap.get(DcMotor.class, "right_back");
        intake     = hardwareMap.get(DcMotor.class, "intake");
        // shooterLeft  = hardwareMap.get(DcMotor.class, "shooter_left");
        // shooterRight = hardwareMap.get(DcMotor.class, "shooter_right");

        // Reverse right side so that positive power moves robot forward
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // Optional: reverse intake if the motor is mounted “backwards”
        // intake.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            // -- GAMEPAD 1 (Driving) --

            // Activate low‐speed mode if right trigger is pressed
            lowSpeedMode = gamepad1.right_trigger > 0.1;
            double speedScale = lowSpeedMode ? 0.3 : 1.0;

            // Read drive and turn inputs
            // forward/backward: right stick Y (invert so up = positive)
            double drive  = -gamepad1.right_stick_y;
            // turning: left stick X
            double turn   =  gamepad1.left_stick_x;

            // Combine for differential drive
            double leftPower  = Range.clip((drive + turn) * speedScale, -1.0, 1.0);
            double rightPower = Range.clip((drive - turn) * speedScale, -1.0, 1.0);

            // Send to the motors
            leftFront.setPower(leftPower);
            leftBack.setPower(leftPower);
            rightFront.setPower(rightPower);
            rightBack.setPower(rightPower);

            // -- GAMEPAD 2 (Intake) --

            // Intake control on left stick Y of gamepad2
            // Push forward to intake (positive), pull back to spit (negative)
            double intakeInput = -gamepad2.left_stick_y;
            // Deadzone to prevent drift
            double intakePower = Math.abs(intakeInput) > 0.05 ? intakeInput : 0.0;
            intake.setPower(intakePower);

            // -- SHOOTER PLACEHOLDERS --
            // shooterLeft.setPower(...);
            // shooterRight.setPower(...);

            // No telemetry requested
        }
    }
}
