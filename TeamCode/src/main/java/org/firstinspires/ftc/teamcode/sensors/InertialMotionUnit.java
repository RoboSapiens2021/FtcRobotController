package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.util.Log;
import org.firstinspires.ftc.teamcode.util.Logger;

public class InertialMotionUnit {
    private static final Log LOG = Logger.getInstance();
    private BNO055IMU bno055IMU;

    private float currentPosition = 0F;
    private float offsetPosition = 0F;
    private BNO055IMU.AccelerationIntegrator accelerationIntegrator = new ImuAccelerationIntegrator();

    public InertialMotionUnit(BNO055IMU bno055IMU) {
        this.bno055IMU = bno055IMU;
    }

    public boolean init() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = false;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = accelerationIntegrator;

        bno055IMU.initialize(parameters);
        // Start the logging of measured acceleration
        bno055IMU.startAccelerationIntegration(new Position(), new Velocity(), 5);
//        LOG.debug("isAccelerometerCalibrated: "+bno055IMU.isAccelerometerCalibrated()+" SystemStatus: "+bno055IMU.getSystemStatus().toShortString());

        return true;
    }

    public float getAngle() {
        Orientation orientation = bno055IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentPosition = orientation.firstAngle;

        return currentPosition - offsetPosition;
    }

    public Acceleration getLinearAcceleration() {
        return accelerationIntegrator.getAcceleration();
    }

    public void resetPosition() {
        offsetPosition = currentPosition;
    }
}
