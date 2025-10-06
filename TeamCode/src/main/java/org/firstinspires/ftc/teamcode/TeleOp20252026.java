package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOp20252026", group = "TeleOp")
public class TeleOp20252026 extends OpMode {

    // Drive motors
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;

    // Intake (single motor)
    private DcMotor intake;

    // Shooter motors (encoder DC motors)
    private DcMotorEx shooterLeft, shooterRight;

    // Servos: standard servo as pusher trigger; continuous-mode smart servo for carousel
    private Servo pusherServo;
    private Servo carouselServo; // using Servo API; continuous-mode smart servo expected to accept position changes

    // Operator variables
    private double shooterTargetRPM = 0.0;
    private boolean shootersAtTarget = false;

    // Low speed mode boolean (driver RT)
    private boolean lowSpeedMode = false;

    // Carousel internal angle (degrees)
    private double carouselAngle = 0.0;

    // Pusher servo positions (0..1). We'll compute relative moves for +/-80 degrees
    private double pusherHomePos = 0.5; // center; tune to your hardware
    private double pusherUpPos = pusherHomePos + 0.2667; // ~80 degrees out of ~300 deg range -> 80/300 = 0.2667
    private boolean pusherBusy = false;
    private ElapsedTime pusherTimer = new ElapsedTime();

    // Constants for encoder conversion
    private static final double ENCODER_PPR = 28.0; // Yellow Jacket reported 28 PPR at output shaft
    private static final double TICKS_PER_REV = ENCODER_PPR; // gear 1:1 assumed

    @Override
    public void init() {
        // Map hardware names - change these to match your robot's configuration file
        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack  = hardwareMap.get(DcMotorEx.class, "rightBack");

        intake = hardwareMap.get(DcMotor.class, "intake");

        shooterLeft  = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");

        pusherServo = hardwareMap.get(Servo.class, "pusherServo");
        carouselServo = hardwareMap.get(Servo.class, "carouselServo");

        // Drive directions: adjust if motors are inverted on your bot
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset encoders for shooter motors and set run mode to use encoders for velocity control
        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Zero-power behavior (coast or brake)
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set initial servo positions
        pusherServo.setPosition(pusherHomePos);
        carouselServo.setPosition(0.5); // mid position baseline; we'll map angles to 0..1

        telemetry.addData("Init", "Complete");
        telemetry.update();
    }

    @Override
    public void start() {
        // nothing special on start
    }

    @Override
    public void loop() {
        // ---------- DRIVER controls (gamepad1) ----------
        // Left stick controls translation (forward/back/strafe) with magnitude-based speed
        double lf = -gamepad1.left_stick_y; // forward/back
        double ls = gamepad1.left_stick_x;  // strafe left/right

        // Right stick controls rotation
        double rotation = gamepad1.right_stick_x; // left negative, right positive

        // Low speed mode if right trigger held (limit to 30%)
        lowSpeedMode = gamepad1.right_trigger > 0.1;
        double speedLimit = lowSpeedMode ? 0.3 : 1.0;

        // Compute mecanum-style mixing: using typical formula for mecanum drive
        double drive = lf;      // forward/back
        double strafe = ls;     // left/right strafing
        double turn = rotation; // rotation component

        double leftFrontPower  = drive + strafe + turn;
        double leftBackPower   = drive - strafe + turn;
        double rightFrontPower = drive - strafe - turn;
        double rightBackPower  = drive + strafe - turn;

        // Normalize wheel powers to keep within [-1,1]
        double max = Math.abs(leftFrontPower);
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(rightBackPower));
        if (max > 1.0) {
            leftFrontPower  /= max;
            leftBackPower   /= max;
            rightFrontPower /= max;
            rightBackPower  /= max;
        }

        // Apply speed limit scaled by joystick magnitude (already scaled by joystick values)
        leftFront.setPower(leftFrontPower * speedLimit);
        leftBack.setPower(leftBackPower * speedLimit);
        rightFront.setPower(rightFrontPower * speedLimit);
        rightBack.setPower(rightBackPower * speedLimit);

        // ---------- OPERATOR controls (gamepad2) ----------
        // Shooter RPM selection
        if (gamepad2.x) {
            shooterTargetRPM = 1000.0;
            setShooterRPM(shooterTargetRPM);
        } else if (gamepad2.a) {
            shooterTargetRPM = 2500.0;
            setShooterRPM(shooterTargetRPM);
        } else if (gamepad2.b) {
            shooterTargetRPM = 5000.0;
            setShooterRPM(shooterTargetRPM);
        } else if (gamepad2.y) {
            shooterTargetRPM = 0.0;
            stopShooter();
        }

        // Update shootersAtTarget boolean using a tolerance (5% error)
        double leftRPM  = readMotorRPM(shooterLeft);
        double rightRPM = readMotorRPM(shooterRight);
        shootersAtTarget = isAtTargetRPM(leftRPM, shooterTargetRPM) && isAtTargetRPM(rightRPM, shooterTargetRPM);

        // RT on gamepad2 triggers pusher servo: rotate up 80 degrees then back down
        if (gamepad2.right_trigger > 0.1 && !pusherBusy) {
            pusherBusy = true;
            // move up
            pusherServo.setPosition(clampServo(pusherUpPos));
            pusherTimer.reset();
        }

        if (pusherBusy) {
            // wait 300 ms at up position then return
            if (pusherTimer.milliseconds() > 300 && pusherTimer.milliseconds() < 700) {
                // return to home
                pusherServo.setPosition(clampServo(pusherHomePos));
            } else if (pusherTimer.milliseconds() >= 700) {
                pusherBusy = false;
                pusherServo.setPosition(clampServo(pusherHomePos));
            }
        }

        // Carousel control using D-pad and LB modifier
        boolean lb = gamepad2.left_bumper;
        boolean dpadRight = gamepad2.dpad_right;
        boolean dpadLeft  = gamepad2.dpad_left;

        if (lb && dpadRight) {
            carouselAngle += 60.0;
            setCarouselAngle(carouselAngle);
        } else if (lb && dpadLeft) {
            carouselAngle -= 60.0;
            setCarouselAngle(carouselAngle);
        } else if (dpadRight) {
            carouselAngle += 120.0;
            setCarouselAngle(carouselAngle);
        } else if (dpadLeft) {
            carouselAngle -= 120.0;
            setCarouselAngle(carouselAngle);
        }

        // Intake (optional) - map to gamepad2 left/right sticks or triggers if you want
        if (gamepad2.left_trigger > 0.1) {
            intake.setPower(gamepad2.left_trigger); // intake in
        } else if (gamepad2.left_bumper) {
            intake.setPower(-0.6); // reverse out
        } else {
            intake.setPower(0);
        }

        // Telemetry: display RPMs and target boolean
        telemetry.addData("Shooter Target RPM", "%.0f", shooterTargetRPM);
        telemetry.addData("Left Shooter RPM", "%.1f", leftRPM);
        telemetry.addData("Right Shooter RPM", "%.1f", rightRPM);
        telemetry.addData("ShootersAtTarget", shootersAtTarget);
        telemetry.addData("LowSpeedMode", lowSpeedMode);
        telemetry.update();
    }

    @Override
    public void stop() {
        stopShooter();
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        intake.setPower(0);
    }

    // ---------- Helper functions ----------

    // Set shooter target velocity in RPM using DcMotorEx.setVelocity (units: ticks per second)
    private void setShooterRPM(double rpm) {
        if (rpm <= 0.0) {
            stopShooter();
            return;
        }
        double ticksPerSec = rpmToTicksPerSecond(rpm);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterLeft.setVelocity(ticksPerSec);
        shooterRight.setVelocity(ticksPerSec);
    }

    private void stopShooter() {
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
    }

    // Convert RPM to encoder ticks per second for DcMotorEx.setVelocity
    private double rpmToTicksPerSecond(double rpm) {
        return (rpm / 60.0) * TICKS_PER_REV;
    }

    // Read RPM from a DcMotorEx using velocity in ticks/sec
    private double readMotorRPM(DcMotorEx motor) {
        // getVelocity returns ticks per second when using an encoder
        double ticksPerSec = motor.getVelocity();
        double rpm = (ticksPerSec / TICKS_PER_REV) * 60.0;
        return rpm;
    }

    // Check whether actual rpm is within tolerance (5% default)
    private boolean isAtTargetRPM(double actualRPM, double targetRPM) {
        if (targetRPM <= 0.0) return actualRPM == 0.0;
        double tol = Math.max(50.0, targetRPM * 0.05); // at least 50 RPM tolerance for low targets
        return Math.abs(actualRPM - targetRPM) <= tol;
    }

    // Map carouselAngle (degrees) to 0..1 servo position
    private void setCarouselAngle(double angleDeg) {
        // Normalize angle to [0,360)
        carouselAngle = ((angleDeg % 360.0) + 360.0) % 360.0;
        // Map 0..360 to 0..1
        double pos = carouselAngle / 360.0;
        carouselServo.setPosition(clampServo(pos));
    }

    private double clampServo(double pos) {
        if (pos < 0) return 0.0;
        if (pos > 1) return 1.0;
        return pos;
    }
}
