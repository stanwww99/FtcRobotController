package org.firstinspires.ftc.teamcode.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "GoBlida5202MotorTest", group = "Examples")
public class GoBlida5202MotorTest extends LinearOpMode {

    // --- CONFIGURE THESE ---
    // Encoder ticks per motor revolution (set this to your encoder CPR)
    private static final int TICKS_PER_MOTOR_REV = 28;
    // Gearbox ratio (motor revolutions : output revolutions). GoBILDA Yellow Jacket 5202 = 99.5:1
    private static final double GEAR_RATIO = 99.5; // from product spec
    // Power used to move motor when going to a target (set conservative value)
    private static final double MOVE_POWER = 0.5;
    // ------------------------

    private DcMotor carousel;

    // current logical angle in degrees in [0,360)
    private double currentAngle = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        carousel = hardwareMap.get(DcMotor.class, "carousel");
        carousel.setDirection(DcMotorSimple.Direction.FORWARD);
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Use encoder and reset. This initial position will be treated as 0 degrees.
        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Compute ticks per output shaft revolution (after gearbox)
        double ticksPerOutputRev = TICKS_PER_MOTOR_REV * GEAR_RATIO;

        waitForStart();

        // After start we treat encoder position 0 as 0 degrees (already reset above)
        currentAngle = 0.0;

        // keep last gamepad state to detect button presses (prevents repeated firing while held)
        boolean lastLb = false;
        boolean lastDpadR = false;
        boolean lastDpadL = false;

        while (opModeIsActive()) {

            boolean lb = gamepad2.left_bumper;
            boolean dpadR = gamepad2.dpad_right;
            boolean dpadL = gamepad2.dpad_left;

            // determine commands by exact priority described:
            // LB + D-Pad R -> +60
            // LB + D-Pad L -> -60
            // D-Pad R -> +120
            // D-Pad L -> -120
            // Only act on *new* presses to move once per press
            if (lb && dpadR && !(lastLb && lastDpadR)) {
                // +60 degrees
                currentAngle = normalizeAngle(currentAngle + 60.0);
                moveToAngle(currentAngle, ticksPerOutputRev);
            } else if (lb && dpadL && !(lastLb && lastDpadL)) {
                // -60 degrees
                currentAngle = normalizeAngle(currentAngle - 60.0);
                moveToAngle(currentAngle, ticksPerOutputRev);
            } else if (dpadR && !lb && !lastDpadR) {
                // +120 degrees
                currentAngle = normalizeAngle(currentAngle + 120.0);
                moveToAngle(currentAngle, ticksPerOutputRev);
            } else if (dpadL && !lb && !lastDpadL) {
                // -120 degrees
                currentAngle = normalizeAngle(currentAngle - 120.0);
                moveToAngle(currentAngle, ticksPerOutputRev);
            } else {
                // No command: ensure motor is not powered and is held (BRAKE)
                carousel.setPower(0.0);
            }

            // update last states
            lastLb = lb;
            lastDpadR = dpadR;
            lastDpadL = dpadL;

            // Small idle to let system breathe
            idle();
        }
    }

    /**
     * Move the carousel to the specified angle (0<=angle<360). Uses RUN_TO_POSITION then stops and holds.
     */
    private void moveToAngle(double angleDegrees, double ticksPerOutputRev) {
        // degrees -> output shaft ticks
        double revolutions = angleDegrees / 360.0;
        int targetTicks = (int)Math.round(revolutions * ticksPerOutputRev);

        // Make sure motor is in RUN_TO_POSITION
        carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        carousel.setTargetPosition(targetTicks);

        // Set power (RUN_TO_POSITION will move motor to target)
        carousel.setPower(MOVE_POWER);

        // Wait until motor reaches target or opmode stops
        while (opModeIsActive() && carousel.isBusy()) {
            // Optionally telemetry for debugging
            telemetry.addData("TargetAngle", angleDegrees);
            telemetry.addData("TargetTicks", targetTicks);
            telemetry.addData("CurTicks", carousel.getCurrentPosition());
            telemetry.update();
            idle();
        }

        // Stop and hold position
        carousel.setPower(0.0);
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Switch back to RUN_USING_ENCODER so further encoder reads are available
        carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Normalize angle into [0, 360)
     */
    private double normalizeAngle(double angle) {
        double a = angle % 360.0;
        if (a < 0) a += 360.0;
        return a;
    }
}
