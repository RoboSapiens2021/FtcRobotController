package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/*
Notes:
       https://learnroadrunner.com/drive-constants.html#ticks-per-rev-max-rpm
        https://www.gobilda.com/strafer-chassis-kit-v4/
 */
public final class DriveConstants {
    private DriveConstants() {
    }

    // TODO: adjust the names of the following hardware devices to match your configuration
    public enum WHEEL_NAME {
        LEFT_FRONT, LEFT_REAR, RIGHT_REAR, RIGHT_FRONT;
    }

    public static final String DEVICE_NAME_IMU = "imu";

    /*
     * Set RUN_USING_ENCODER to true to enable built-in hub velocity control using drive encoders.
     * Set this flag to false if drive encoders are not present and an alternative localization
     * method is in use (e.g., tracking wheels).
     */
    public static final boolean RUN_USING_ENCODER = true;

    /*
     * These are motor constants that should be listed online for your motors.
     */
    public static final double TICKS_PER_REV = 537.7D;
    public static final double MAX_RPM = 1;

    /*
     * These are physical constants that can be determined from your robot (including the track width).
     * The values were selected with inches in mind.
     */
    public static final double GEAR_RATIO = 19.2D;// output (wheel) speed / input (motor) speed
    public static double WHEEL_RADIUS_INCHES = 2; // in
    public static double TRACK_WIDTH_INCHES = 1; // in
    public static final double WHEEL_DIAMETER_INCHES = 3.77953D;
    public static final double TICKS_PER_INCH = (TICKS_PER_REV * GEAR_RATIO) / (WHEEL_DIAMETER_INCHES * Math.PI);
    public static final double INCH_PER_TICK = 1 / TICKS_PER_INCH;

    public static final double DRIVE_SPEED = 0.6D;
    public static final double TURN_SPEED = 0.5D;
    public static final double ZERO_POWER = 0.0D;

    /*
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     */
    public static double kV = 1.0 / rpmToVelocity(MAX_RPM);
    public static double kA = 0;
    public static double kStatic = 0;

    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. It is a good idea for testing to start
     * small and gradually increase them later after everything is working. All distance units are
     * inches.
     */
    public static double MAX_VEL = 30;
    public static double MAX_ACCEL = 30;
    public static double MAX_ANG_VEL = 60;
    public static double MAX_ANG_ACCEL = 60;

    public static double encoderTicksToInches(double ticks) {
        return ticks * INCH_PER_TICK;
    }

    public static double inchesToEncoderTicks(double inches) {
        return inches * TICKS_PER_INCH;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * Math.PI * WHEEL_DIAMETER_INCHES / 60.0;
    }

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0, getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }
}
