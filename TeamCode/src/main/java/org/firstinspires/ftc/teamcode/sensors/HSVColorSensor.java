package org.firstinspires.ftc.teamcode.sensors;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Logger;

public class HSVColorSensor {
    private final Logger logger = Logger.getInstance();


    private NormalizedColorSensor colorSensor;
    private float gain = 2;

    public HSVColorSensor(NormalizedColorSensor sensor) {
        colorSensor = sensor;
    }

    public void init() {
        colorSensor.setGain(gain);
    }

    public void setGain(float gain) {
        this.gain = gain;
        colorSensor.setGain(gain);
    }

    public void toggleLED() {
        if (colorSensor instanceof SwitchableLight) {
            SwitchableLight light = (SwitchableLight) colorSensor;
            light.enableLight(!light.isLightOn());
        } else {
            logger.warn("SwitchableLight is not available");
        }
    }

    public double getDistance() {
        double distance = 0D;
        if (colorSensor instanceof DistanceSensor) {
            distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.INCH);
            if (distance == DistanceSensor.distanceOutOfRange) {
                distance = 0D;
                logger.warn("DistanceSensor is not available");
            }
        } else {
            logger.warn("DistanceSensor is not available");
        }
        return distance;
    }

    public float[] getHSV() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        final float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);

        return hsvValues;

    }

    public float[] getRGB() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        final float[] rgbValues = new float[4];
        rgbValues[0] = colors.red;
        rgbValues[1] = colors.green;
        rgbValues[2] = colors.blue;
        rgbValues[3] = colors.alpha;

        return rgbValues;
    }

//    public boolean isColorRed(){
//        float[] hsvValues = getHSV();
//        if(hsvValues[0]){
//
//        }
//
//    }


}
