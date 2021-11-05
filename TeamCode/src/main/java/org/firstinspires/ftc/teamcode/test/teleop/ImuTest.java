package org.firstinspires.ftc.teamcode.test.teleop;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.util.Constants;

@TeleOp(name = "ImuTest", group = "test")
public class ImuTest extends LinearOpMode {
    private BNO055IMU bno055IMU;

    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU bno055IMU = hardwareMap.get(BNO055IMU.class, Constants.DEVICE_NAME.imu.name());

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
//        NaiveAccelerationIntegrator();
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        bno055IMU.initialize(parameters);
        // Start the logging of measured acceleration
        bno055IMU.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        waitForStart();

        while (opModeIsActive()) {
            logPosition();
        }
    }

    public void logPosition() {
        Orientation orientation = bno055IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addData("Angle Z: ", orientation.firstAngle);
        telemetry.addData("Angle Y: ", orientation.secondAngle);
        telemetry.addData("Angle X: ", orientation.thirdAngle);
        telemetry.addLine();

        Acceleration acceleration = bno055IMU.getAcceleration();
        telemetry.addData("Accel X: ", acceleration.xAccel);
        telemetry.addData("Accel Y: ", acceleration.yAccel);
        telemetry.addData("Accel Z: ", acceleration.zAccel);

        telemetry.update();
    }


}
