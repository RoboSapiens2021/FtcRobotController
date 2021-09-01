package org.firstinspires.ftc.teamcode.util;

public final class Constants {
    private Constants() {
    }

    public enum WHEEL_NAME {
        FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT;
    }
/*
Notes:
       https://learnroadrunner.com/drive-constants.html#ticks-per-rev-max-rpm
        https://www.gobilda.com/strafer-chassis-kit-v4/
 */
    public static final double COUNTS_PER_MOTOR_REV = 537.7D;
    public static final double DRIVE_GEAR_REDUCTION = 19.2D;
    public static final double WHEEL_DIAMETER_INCHES = 3.77953D;
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    public static final double DRIVE_SPEED = 0.6D;
    public static final double TURN_SPEED = 0.5D;

    public static final double ZERO_POWER = 0.0D;

    //TODO
    public static final int AXLE_DISTANCE_IN_INCHES = 4;
}
