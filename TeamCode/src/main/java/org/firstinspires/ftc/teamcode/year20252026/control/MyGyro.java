package org.firstinspires.ftc.teamcode.year20252026.control;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class MyGyro {

    // Stores the last heading the robot believed it had.
    // Useful for preserving orientation between OpModes or during IMU resets.
    public static double lastKnownHeading = 0.0;

    // The IMU object shared across the robot codebase.
    // Static so multiple subsystems can reference the same sensor instance.
    public static IMU imu;

    /**
     * Creates and initializes the IMU with the correct physical mounting orientation.
     *
     * FTC IMU readings depend heavily on how the Control Hub or Expansion Hub
     * is mounted on the robot. The IMU must be told:
     *   - Which direction the REV logo faces
     *   - Which direction the USB port faces
     *
     * Without this, heading, pitch, and roll will be incorrect.
     *
     * @param hardwareMap the robot's hardware map
     * @return the initialized IMU instance
     */
    public static IMU createIMU(HardwareMap hardwareMap){

        // Retrieve IMU from hardware configuration.
        imu = hardwareMap.get(IMU.class, "imu");

        /*
         * Configure the IMU orientation:
         *
         * RevHubOrientationOnRobot.LogoFacingDirection.RIGHT:
         *     The REV logo on the Hub is facing the robot's RIGHT side.
         *
         * RevHubOrientationOnRobot.UsbFacingDirection.UP:
         *     The USB port on the Hub is pointing UP toward the sky.
         *
         * These two parameters define the 3D orientation of the Hub.
         * The IMU uses this to correctly interpret yaw (heading),
         * pitch (forward tilt), and roll (side tilt).
         *
         * If these values do not match the physical mounting,
         * the robot will turn incorrectly or drift during autonomous.
         */
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));

        return imu;
    }
}
