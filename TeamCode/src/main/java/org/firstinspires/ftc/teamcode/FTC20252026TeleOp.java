package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "FTC20252026TeleOp", group = "TeleOp")
public class FTC20252026TeleOp extends OpMode {

    // Drive motors (two left, two right)
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;
    // Intake motor
    private DcMotor intakeMotor;
    // Shooter motors (with encoders)
    private DcMotorEx shooter1, shooter2;
    // Servos (mapping to be filled once names are known)
    private Servo continuousServo, normalServo;

    // Constants
    private static final double LOW_SPEED_FACTOR = 0.3;
    private static final double TICKS_PER_REV = 28;      // Encoder pulses per motor revolution
    private static final double TARGET_RPM   = 5000.0;  // Desired shooter speed

    // State
    private boolean lowSpeedMode = false;

    @Override
    public void init() {
        // Map drive motors
        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack  = hardwareMap.get(DcMotorEx.class, "rightBack");

        // Reverse left side to ensure forward is positive
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // Use run-without-encoder for smooth TeleOp driving
        leftFront .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack  .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Map intake motor
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Map shooter motors and enable encoder-based velocity control
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Declare servos (to be mapped)
        continuousServo = hardwareMap.get(Servo.class, "continuousServo");
        normalServo     = hardwareMap.get(Servo.class, "normalServo");
    }

    @Override
    public void loop() {
        // ----- DRIVER (gamepad1) -----
        double drive = -gamepad1.right_stick_y;     // forward/backwards
        double turn  =  gamepad1.left_stick_x;      // left/right rotation

        // Low-speed “turtle” mode when RT is held
        lowSpeedMode = gamepad1.right_trigger > 0.0;
        double speedFactor = lowSpeedMode ? LOW_SPEED_FACTOR : 1.0;

        double leftPower  = (drive + turn) * speedFactor;
        double rightPower = (drive - turn) * speedFactor;

        leftFront .setPower(leftPower);
        leftBack  .setPower(leftPower);
        rightFront.setPower(rightPower);
        rightBack .setPower(rightPower);

        // ----- OPERATOR (gamepad2) -----
        // Intake: forward/back via left joystick vertical axis
        double intakePower = -gamepad2.left_stick_y;
        intakeMotor.setPower(intakePower);

        // Shooter: spin up to TARGET_RPM when RT is pressed
        if (gamepad2.right_trigger > 0.0) {
            // Convert RPM to ticks/sec: (RPM / 60) * TICKS_PER_REV
            double targetVelocity = (TARGET_RPM / 60.0) * TICKS_PER_REV;
            shooter1.setVelocity(targetVelocity);
            shooter2.setVelocity(targetVelocity);
        } else {
            shooter1.setPower(0.0);
            shooter2.setPower(0.0);
        }

        // ----- TELEMETRY -----
        // Convert current velocity (ticks/sec) back to RPM
        double rpm1 = (shooter1.getVelocity() * 60.0) / TICKS_PER_REV;
        double rpm2 = (shooter2.getVelocity() * 60.0) / TICKS_PER_REV;
        boolean atSpeed = rpm1 >= TARGET_RPM && rpm2 >= TARGET_RPM;

        telemetry.addData("Drive Mode", lowSpeedMode ? "Low (30%)" : "Full (100%)");
        telemetry.addData("Shooter RPM1", "%.0f", rpm1);
        telemetry.addData("Shooter RPM2", "%.0f", rpm2);
        telemetry.addData("At 5000+ RPM", atSpeed);
        telemetry.update();
    }
}
