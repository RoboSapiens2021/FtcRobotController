package org.firstinspires.ftc.teamcode.test.teleop;


import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.teamcode.util.Constants.WHEEL_NAME.LEFT_FRONT;
import static org.firstinspires.ftc.teamcode.util.Constants.WHEEL_NAME.LEFT_REAR;
import static org.firstinspires.ftc.teamcode.util.Constants.WHEEL_NAME.RIGHT_FRONT;
import static org.firstinspires.ftc.teamcode.util.Constants.WHEEL_NAME.RIGHT_REAR;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Logger;

@TeleOp(name = "ImuTest", group = "test")
public class ImuTest extends LinearOpMode {
    private static Logger logger = Logger.getInstance();

    private BNO055IMU bno055IMU;
    private DcMotorEx motorLeftFront;
    private DcMotorEx motorLeftRear;
    private DcMotorEx motorRightRear;
    private DcMotorEx motorRightFront;

    private float currentPosition = 0F;
    private float offsetPosition = 0F;

    @Override
    public void runOpMode() throws InterruptedException {
        logger.setTelemetry(telemetry);

        initMotors();

        logger.debug("Initializing IMU");
        BNO055IMU bno055IMU = hardwareMap.get(BNO055IMU.class, Constants.DEVICE_NAME.imu.name());

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        bno055IMU.initialize(parameters);
        logger.debug("Initialized IMU");

        waitForStart();

        while (opModeIsActive()) {
            logger.info(getCurrentPosition());
            turn(90);
            logger.info(getCurrentPosition());
            turn(90);
            logger.info(getCurrentPosition());
            resetPosition();
            turn(-90);
            logger.info(getCurrentPosition());
            turn(-90);
            logger.info(getCurrentPosition());
        }
    }

    public float getCurrentPosition() {
        Orientation orientation = bno055IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentPosition = orientation.firstAngle;

        return currentPosition - offsetPosition;
    }

    private void resetPosition() {
        offsetPosition = currentPosition;
    }

    private void turn(int degrees) {
        float angularOrientation = getCurrentPosition();
        setMotorPowers(-0.2D, -0.2D, 0.2, 0.2);
        while (angularOrientation < degrees && !isStopRequested()) {
            angularOrientation = getCurrentPosition();
            logger.debug(angularOrientation);
        }
        setMotorPowers(0D, 0D, 0D, 0D);
    }

    private void initMotors() {
        motorLeftFront = hardwareMap.get(DcMotorEx.class, LEFT_FRONT.name());
        motorLeftRear = hardwareMap.get(DcMotorEx.class, LEFT_REAR.name());
        motorRightRear = hardwareMap.get(DcMotorEx.class, RIGHT_REAR.name());
        motorRightFront = hardwareMap.get(DcMotorEx.class, RIGHT_FRONT.name());

        motorLeftFront.setZeroPowerBehavior(BRAKE);
        motorLeftRear.setZeroPowerBehavior(BRAKE);
        motorRightRear.setZeroPowerBehavior(BRAKE);
        motorRightFront.setZeroPowerBehavior(BRAKE);
    }


    private void setMotorPowers(double v1, double v2, double v3, double v4) {
        motorLeftFront.setPower(v1);
        motorLeftRear.setPower(v2);
        motorRightRear.setPower(v3);
        motorRightFront.setPower(v4);
    }

}
