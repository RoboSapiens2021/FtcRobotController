package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.util.Log;
import org.firstinspires.ftc.teamcode.util.Logger;

public class QualcommColorSensor {

    private ColorSensor colorSensor;
    private boolean isAvailable;
    private static final int DEFAULT_CHANNEL_VALUE = 0;
    private final Logger logger = Logger.getInstance();

    public QualcommColorSensor(ColorSensor sensor) {
        colorSensor = sensor;
        this.isAvailable = null != sensor;
        logger.warn("QualcommColorSensor is not available");
    }


    public void init() {
        if (isAvailable) {

        }
    }

    /**
     * Turn the LED on
     */
    public void enable() {
        if (isAvailable) {
            colorSensor.enableLed(true);
        }
    }

    /**
     * Turn the LED off
     */
    public void disable() {
        if (isAvailable) {
            colorSensor.enableLed(false);
        }
    }

    /**
     * Red channel value
     *
     * @return
     */
    public int getRedValue() {
        if (isAvailable) {
            return colorSensor.red();
        }
        return DEFAULT_CHANNEL_VALUE;
    }

    public int getGreenValue() {
        if (isAvailable) {
            return colorSensor.green();
        }
        return DEFAULT_CHANNEL_VALUE;
    }

    public int getBlueValue() {
        if (isAvailable) {
            return colorSensor.blue();
        }
        return DEFAULT_CHANNEL_VALUE;
    }

    public int getAlphaValue() {
        if (isAvailable) {
            return colorSensor.alpha();
        }
        return DEFAULT_CHANNEL_VALUE;
    }

    public int getCombinedColorValue() {
        if (isAvailable) {
            return colorSensor.argb();
        }
        return DEFAULT_CHANNEL_VALUE;
    }
}
