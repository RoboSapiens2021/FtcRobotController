package org.firstinspires.ftc.teamcode.util;

public class Utils {
    public static final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public static final double intToDouble(int x) {
        return Integer.valueOf(x).doubleValue();
    }

    public static final int doubleToInt(double x) {
        return Double.valueOf(x).intValue();
    }

}
