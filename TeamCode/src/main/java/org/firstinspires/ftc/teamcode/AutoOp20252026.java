package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@Autonomous(name = "AutoOp20252026", group = "Autonomous")
public class AutoOp20252026 extends LinearOpMode {

    // Drive motors
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;

    // Additional motors
    private DcMotor intake = null;
    private DcMotor shooterLeft = null;
    private DcMotor shooterRight = null;

    // Servos (one continuous rotation, one standard)
    private CRServo carouselContinuous = null;
    private Servo shooterServo = null;

    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware initialization - change names here to match your robot config if needed
        leftFront   = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack    = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront  = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack   = hardwareMap.get(DcMotor.class, "rightBack");

        intake      = hardwareMap.get(DcMotor.class, "intake");
        shooterLeft = hardwareMap.get(DcMotor.class, "shooterLeft");
        shooterRight= hardwareMap.get(DcMotor.class, "shooterRight");

        carouselContinuous = hardwareMap.get(CRServo.class, "carouselServo");
        shooterServo        = hardwareMap.get(Servo.class, "pusherServo");

        // Set motor directions: flip right side so positive power -> forward
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // Stop and configure run mode for all motors (initialized even if unused)
        DcMotor[] allDcMotors = new DcMotor[] {
                leftFront, leftBack, rightFront, rightBack,
                intake, shooterLeft, shooterRight
        };

        for (DcMotor m : allDcMotors) {
            m.setPower(0);
            try {
                m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } catch (Exception e) {
                // Some motors may not have encoders; ignore mode-setting errors
            }
        }

        // Set initial servo positions / stop continuous servo
        try {
            shooterServo.setPosition(0.5); // neutral/mid for a standard servo
        } catch (Exception ignored) { }

        try {
            carouselContinuous.setPower(0); // stop CRServo
        } catch (Exception ignored) { }

        telemetry.addData("Status", "Hardware initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Autonomous action: drive forward straight for 1.5 seconds using only the 4 drive motors
        double drivePower = 0.5; // adjust as needed for consistent movement
        leftFront.setPower(drivePower);
        leftBack.setPower(drivePower);
        rightFront.setPower(drivePower);
        rightBack.setPower(drivePower);

        sleep(1500); // 1.5 seconds

        // Stop all drive motors and hold standstill
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        // Leave other hardware in safe idle state
        intake.setPower(0);
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
        try { carouselContinuous.setPower(0); } catch (Exception ignored) { }
        try { shooterServo.setPosition(0.5); } catch (Exception ignored) { }

        telemetry.addData("Autonomous", "Completed: moved forward 1.5s and stopped");
        telemetry.update();

        // Keep opmode alive until stop requested so robot stays in standstill
        while (opModeIsActive()) {
            idle();
        }
    }
}
