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

    /**
     * Converts degrees to work with sine and cosine.
     * The equation were made in Desmos by plotting certain points (input,output)
     * Equation 1: y = -x + 90
     * Equation 2: y = -x + 450
     *
     * @param heading robot heading in degrees
     * @return the fixed heading in degrees
     */
    public static double cvtDegrees(double heading) {
        if (heading >= 0D && heading < 90D) {
            return -heading + 90;
        }
        return -heading + 450;
    }

    public static double max(double x1, double x2, double x3, double x4) {
        double max1 = Math.max(x1, x2);
        double max2 = Math.max(x3, x4);

        return Math.max(max1, max2);

    }

}
