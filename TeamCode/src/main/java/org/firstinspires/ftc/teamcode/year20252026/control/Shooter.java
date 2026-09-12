package org.firstinspires.ftc.teamcode.year20252026.control;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Shooter subsystem:
 * - Controls a high‑speed flywheel shooter (GoBilda 5202 motor, 1:1)
 * - Controls a pusher servo that feeds rings/pixels into the flywheel
 *
 * This class abstracts RPM control, servo angle mapping, and shooter state.
 */
public class Shooter {

    // High‑speed flywheel motor (GoBilda 5202, 6000 RPM, 1:1 gearbox)
    private DcMotorEx shooter;

    // Servo that pushes game elements into the flywheel
    private Servo pusher;

    // Encoder resolution for GoBilda 5202 motor (1:1 gearbox)
    // 28 ticks per revolution at the output shaft.
    public static final double SHOOTER_PPR = 28.0;

    // Servo range (in degrees) for Studica Smart Servo in standard mode.
    // Maps 0–300 degrees → 0.0–1.0 servo position.
    private static final double SERVO_FULL_RANGE_DEG = 300.0;

    // RPM tracking
    private double currentRPM = 0.0;   // measured RPM from encoder
    private double targetRPM = 0.0;    // desired RPM set by user

    // State flags
    private boolean shooterActive;
    private boolean pusherActive;

    /**
     * Constructor: initializes shooter motor and pusher servo.
     *
     * @param hardwareMap FTC hardware map
     */
    public Shooter(HardwareMap hardwareMap){
        shooter  = hardwareMap.get(DcMotorEx.class, "shooter");
        pusher   = hardwareMap.get(Servo.class, "pusher");

        // Reverse direction so positive velocity spins flywheel forward.
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        // Use encoder feedback for velocity control.
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // FLOAT allows the flywheel to spin down naturally when power = 0.
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Initialize pusher servo to 0 degrees (safe retracted position).
        setPusherAngle(0.0);
    }

    /**
     * Sets the pusher servo to a specific angle in degrees.
     * Converts degrees → normalized servo position (0.0–1.0).
     *
     * @param angleDeg desired servo angle (0–300 degrees)
     */
    public void setPusherAngle(double angleDeg) {
        double pos = RangeClip(angleDeg / SERVO_FULL_RANGE_DEG, 0.0, 1.0);
        pusher.setPosition(pos);
    }

    /**
     * Clips a value to a given range.
     */
    private double RangeClip(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    /**
     * Sets the target RPM for the shooter flywheel.
     * Converts RPM → ticks per second and commands motor velocity.
     *
     * @param desiredRPM desired flywheel speed
     */
    public void setTargetRPM(double desiredRPM) {
        targetRPM = desiredRPM;

        // Convert RPM → ticks/sec:
        // ticks/sec = (RPM * ticks/rev) / 60
        double ticksPerSec = desiredRPM * SHOOTER_PPR / 60.0;

        // DcMotorEx supports velocity control directly in ticks/sec.
        shooter.setVelocity(ticksPerSec);

        // Update currentRPM immediately after commanding new velocity.
        updateRPM();
    }

    /**
     * Updates currentRPM using encoder velocity.
     * Converts ticks/sec → RPM.
     */
    public void updateRPM() {
        double ticksPerSec = shooter.getVelocity();
        currentRPM = ticksPerSec * 60.0 / SHOOTER_PPR;
    }

    /**
     * Starts the shooter at a desired RPM.
     * Pusher remains inactive until RPM target is reached.
     */
    public void start(double desiredRPM){
        setTargetRPM(desiredRPM);
        shooterActive = true;
        pusherActive = false;
    }

    /**
     * Stops shooter and retracts pusher.
     */
    public void stop(){
        setTargetRPM(0);
        setPusherAngle(0);
        shooterActive = false;
        pusherActive = false;
    }

    /**
     * Activates pusher to feed a game element into the flywheel.
     * 80 degrees is a typical “push forward” angle.
     */
    public void push(){
        pusherActive = true;
        setPusherAngle(80.0);
    }

    /**
     * Checks whether the shooter has reached its target RPM.
     *
     * @return true if currentRPM >= targetRPM
     */
    public boolean isTargetMet(){
        return currentRPM >= targetRPM;
    }

    public double getCurrentRPM(){
        return currentRPM;
    }

    public boolean isShooterActive(){
        return shooterActive;
    }

    public boolean isPusherActive(){
        return pusherActive;
    }

    public DcMotorEx getMotor(){
        return shooter;
    }

    public double getTargetRPM() {
        return targetRPM;
    }
}
