package org.firstinspires.ftc.teamcode.year20252026.control;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Carousel {

    /**
     * RUN_TO_POSITION mode (autonomous rotation)
     * The motor uses its internal PID controller to move to a target encoder position.
     */
    public static final int AUTO = 0;

    /**
     * RUN_USING_ENCODER mode (manual control)
     * The motor applies raw power while still reporting encoder values.
     */
    public static final int MANUAL = 1;

    // The carousel motor (GoBilda 60 RPM motor with 99.5:1 gearbox)
    // DcMotorEx gives access to advanced features like setTargetPositionTolerance().
    private DcMotorEx carousel;

    // Tracks the logical "current position" we believe the carousel should be at.
    // This mirrors the encoder but is updated only when we command movement.
    private int position = 0;

    // Encoder counts per full rotation of the carousel output shaft.
    // 2786.2 PPR is typical for a GoBilda 5202/5203 motor with a 99.5:1 gearbox.
    private static final double CAROUSEL_PPR = 2786.2;

    // Precomputed encoder distances for 1/3 and 1/6 rotations.
    private static final double CAROUSEL_PPR3rd = CAROUSEL_PPR / 3;
    private static final double CAROUSEL_PPR6th = CAROUSEL_PPR / 6;

    // Tracks whether a RUN_TO_POSITION movement is currently active.
    private boolean rotateActive = false;

    /**
     * Constructor: initializes the carousel motor and resets its encoder.
     * @param hardwareMap FTC hardware map
     * @param mode AUTO or MANUAL (currently unused but kept for future expansion)
     */
    public Carousel(HardwareMap hardwareMap, int mode){
        carousel = hardwareMap.get(DcMotorEx.class, "carousel");

        // Reset encoder so position = 0 corresponds to the starting angle.
        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // BRAKE ensures the carousel does not drift when power = 0.
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        position = 0; // Encoder is zeroed above.
    }

    /**
     * Returns the underlying motor object for advanced control or telemetry.
     */
    public DcMotorEx getMotor(){
        return carousel;
    }

    // --- Convenience rotation commands (1/3 and 1/6 turns) ---

    public void rotateThirdRight(){
        // Negative direction = clockwise (depending on motor orientation)
        rotateCarousel(-(int)CAROUSEL_PPR3rd);
    }

    public void rotateThirdLeft(){
        rotateCarousel((int)CAROUSEL_PPR3rd);
    }

    public void rotateSixthRight(){
        rotateCarousel(-(int)CAROUSEL_PPR6th);
    }

    public void rotateSixthLeft(){
        rotateCarousel((int)CAROUSEL_PPR6th);
    }

    /**
     * Core rotation method: moves carousel by a specific encoder amount.
     * Uses RUN_TO_POSITION mode with full power.
     *
     * @param amt encoder ticks to move (positive = left, negative = right)
     */
    public void rotateCarousel(int amt){
        int newPosition = position + amt;   // Compute new target position
        position = newPosition;             // Update logical position tracker

        carousel.setTargetPosition(newPosition);
        carousel.setTargetPositionTolerance(5); // ±5 ticks tolerance
        carousel.setPower(1);                   // Full power
        carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rotateActive = true;
    }

    /**
     * Same as rotateCarousel(), but allows custom power.
     */
    public void rotateCarousel(int amt, double power){
        int newPosition = position + amt;
        position = newPosition;

        carousel.setTargetPosition(newPosition);
        carousel.setTargetPositionTolerance(5);
        carousel.setPower(power);
        carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rotateActive = true;
    }

    /**
     * Returns the logical position (not necessarily the live encoder value).
     * This is the target position we last commanded.
     */
    public int getPosition(){
        return position;
    }

    /**
     * Returns true if a RUN_TO_POSITION movement is currently active.
     */
    public boolean isRotateActive(){
        return rotateActive;
    }

    /**
     * Manual movement: applies raw power and updates the logical position
     * to match the live encoder reading.
     *
     * Useful for TeleOp or manual override.
     */
    public void move(double power){
        carousel.setPower(power);
        position = carousel.getCurrentPosition();
    }

    /**
     * Switches between AUTO (RUN_TO_POSITION) and MANUAL (RUN_USING_ENCODER).
     */
    public void setMode(int mode){
        if (mode == AUTO)
            carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        else if (mode == MANUAL)
            carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Stops the carousel immediately:
     * - Clears rotateActive flag
     * - Sets power to zero
     * - Returns motor to manual mode
     */
    public void stop() {
        rotateActive = false;
        move(0);
        setMode(Carousel.MANUAL);
    }

    /**
     * Checks whether the carousel has finished its RUN_TO_POSITION movement.
     *
     * @return true if:
     *         - a rotation was active, AND
     *         - the motor is no longer busy (target reached)
     *
     *         false if:
     *         - rotation was never active, OR
     *         - the motor is still moving
     */
    public boolean isFinished(){
        return rotateActive && !carousel.isBusy();
    }
}
