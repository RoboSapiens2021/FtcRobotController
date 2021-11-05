package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class InertialMotionUnit {
    private BNO055IMU bno055IMU;

    private float currentPosition = 0F;
    private float offsetPosition = 0F;

    public InertialMotionUnit(BNO055IMU bno055IMU) {
        this.bno055IMU = bno055IMU;
    }

    public boolean init() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
//        NaiveAccelerationIntegrator();
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        bno055IMU.initialize(parameters);
        // Start the logging of measured acceleration
        bno055IMU.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        return true;
    }

    public String getStatus() {
        return "calib: " + bno055IMU.getCalibrationStatus().toString() + "status: " + bno055IMU.getSystemStatus().toShortString();
    }

    public float getAngle() {
        Orientation orientation = bno055IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentPosition = orientation.firstAngle;

        return currentPosition - offsetPosition;
    }

    public void resetPosition() {
        offsetPosition = currentPosition;
    }
}
