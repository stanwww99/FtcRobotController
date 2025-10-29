package org.firstinspires.ftc.teamcode.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "GoBlida5202RPMTest", group = "TeleOp")
public class GoBlida5202RPMTest extends LinearOpMode {

    // Adjust this based on your encoder counts per output shaft revolution.
    // goBILDA 5202 (1:1) has 28 PPR at the output shaft; depending on how the FTC SDK counts
    // (edges vs. quadrature decoding), you may observe 28, 56, or 112 counts per revolution.
    // Start with 28; if measured RPM is exactly 1/4 of expected, bump to 112.
    private static final double TICKS_PER_REV = 28.0;

    // Target power to test (0.0 to 1.0). You can change this on the gamepad if desired.
    private static final double TEST_POWER = 0.5;

    // Sampling window (seconds) for RPM calculation; shorter windows are noisier.
    private static final double SAMPLE_WINDOW_SEC = 0.250; // 250 ms

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor motor = hardwareMap.get(DcMotor.class, "shooter"); // Configure name in Robot Configuration

        // Reset encoder ONCE at the beginning
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Ready. Encoder reset to 0.");
        telemetry.addData("Ticks per rev (config)", TICKS_PER_REV);
        telemetry.update();

        waitForStart();

        // Apply the desired test power
        motor.setPower(TEST_POWER);

        // Timing helpers
        ElapsedTime sampleTimer = new ElapsedTime();
        int lastPos = motor.getCurrentPosition();
        double lastTime = sampleTimer.seconds();

        while (opModeIsActive()) {
            // Sample over a fixed window to reduce jitter
            if (sampleTimer.seconds() - lastTime >= SAMPLE_WINDOW_SEC) {
                int currentPos = motor.getCurrentPosition();
                double currentTime = sampleTimer.seconds();

                int deltaTicks = currentPos - lastPos;
                double deltaTime = currentTime - lastTime;

                // Revolutions in sample window
                double revs = deltaTicks / TICKS_PER_REV;

                // RPM = revs per minute
                double rpm = (revs / deltaTime) * 60.0;

                telemetry.addData("Power", TEST_POWER);
                telemetry.addData("Delta ticks", deltaTicks);
                telemetry.addData("Sample window (s)", deltaTime);
                telemetry.addData("RPM", String.format("%.1f", rpm));
                telemetry.update();

                // Prepare for next window
                lastPos = currentPos;
                lastTime = currentTime;
            }

            // Optional: allow dynamic power tuning with gamepad
            if (gamepad1.dpad_up) motor.setPower(Math.min(1.0, motor.getPower() + 0.05));
            if (gamepad1.dpad_down) motor.setPower(Math.max(0.0, motor.getPower() - 0.05));
            if (gamepad1.a) motor.setPower(0.0);
        }

        // Stop motor at the end
        motor.setPower(0.0);
    }
}
