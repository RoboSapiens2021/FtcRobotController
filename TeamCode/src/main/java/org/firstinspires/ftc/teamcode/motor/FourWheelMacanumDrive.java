package org.firstinspires.ftc.teamcode.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Log;
import org.firstinspires.ftc.teamcode.util.Logger;
import org.firstinspires.ftc.teamcode.util.Utils;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class FourWheelMacanumDrive extends AbstractDriveTrain {
    private static final Log LOG = Logger.getInstance();

    private DcMotorEx motorLeftFront;
    private DcMotorEx motorLeftRear;
    private DcMotorEx motorRightRear;
    private DcMotorEx motorRightFront;

    private List<DcMotorEx> motors;

    public FourWheelMacanumDrive(DcMotorEx motorLeftFront,
                                 DcMotorEx motorLeftRear,
                                 DcMotorEx motorRightRear,
                                 DcMotorEx motorRightFront) {
        this.motorLeftFront = motorLeftFront;
        this.motorLeftRear = motorLeftRear;
        this.motorRightRear = motorRightRear;
        this.motorRightFront = motorRightFront;

        motors = Arrays.asList(motorLeftFront, motorLeftRear, motorRightRear, motorRightFront);
    }

    public void init() {
        setDirection(motorLeftFront, DcMotor.Direction.REVERSE);
        setDirection(motorLeftRear, DcMotor.Direction.REVERSE);
//        setDirection(motorRightRear, DcMotor.Direction.REVERSE);
//        setDirection(motorRightFront, DcMotor.Direction.REVERSE);

        for (DcMotorEx motor : motors) {
            //Motor encoders are set to 0.85 by default for safety
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            //changing this to 1.0 can make you more vulnerable to changes in speed when your battery gets low, so make sure to use a PID
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);

            setZeroPowerBehavior(motor, DcMotor.ZeroPowerBehavior.BRAKE);

            resetEncoder();

//            if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
//                setPIDFCoefficients(motor, DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
//            }

        }

    }


    public void resetEncoder() {
        for (DcMotorEx motor : motors) {
            setMode(motor, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(getWheelPosition(motor));
        }
        return wheelPositions;
    }

    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(getWheelVelocity(motor));
        }
        return wheelVelocities;
    }

    public void setMotorPowers(double v1, double v2, double v3, double v4) {
        setMotorPower(motorLeftFront, v1);
        setMotorPower(motorLeftRear, v2);
        setMotorPower(motorRightRear, v3);
        setMotorPower(motorRightFront, v4);
    }

    private void setMotorPowers(double power) {
        for (DcMotorEx motor : motors) {
            setMotorPower(motor, power);
        }
    }

    private void stopMotors() {
        for (DcMotorEx motor : motors) {
            stopMotor(motor);
        }
    }

    public void setMotorMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            setMode(motor, runMode);
        }
    }

    public void driveByTime(double power, long time) {
        setMotorPowers(power);

        Utils.sleep(time);

        setMotorPowers(Constants.ZERO_POWER);
    }


    /**
     * @param speed + to drive forward and - to drive backward
     * @param time
     */
    public void strafeStraight(double speed, long time) {
        float targetAngle = imu.getAngle();
        long startTime = System.currentTimeMillis();

        while (isOpModeActive() && (System.currentTimeMillis() - startTime) < time) {
            float currentAngle = imu.getAngle();
            float angleDifference = currentAngle - targetAngle;
            if (angleDifference < -180) {
                angleDifference = angleDifference + 360;
            } else if (angleDifference > 180) {
                angleDifference = 360 - angleDifference;
            }

            if (speed > 0D) { //Drive Forward
                if (angleDifference > 1) { //if the robot is strafing right and needs to turn to the right, reduce the speed of the back motors
                    setMotorPowers(speed, -speed + 0.2, speed - 0.2, -speed);
                } else if (angleDifference < 1) {//if the robot is strafing right and needs to turn to the left, reduce the speed of the front motors
                    setMotorPowers(speed - 0.2, -speed, speed, -speed + 0.2);
                } else {
                    setMotorPowers(speed);
                }
            } else {
//                if (angleDifference < -1) {
//                    setMotorPowers(speed, -speed + 0.2, speed - 0.2, -speed);
//                } else if (angleDifference < 1) {
//                    setMotorPowers(speed - 0.2, -speed, speed, -speed + 0.2);
//                } else {
//                    setMotorPowers(speed);
//                }
            }
        }
        stopMotors();
    }

//    public void drive(float leftX, float leftY, float rightX) {
////        double r = Math.hypot(leftX, leftY);
////        double robotAngle = Math.atan2(-leftY, leftX) - (Math.PI / 4);
////
////        double cosine = Math.cos(robotAngle);
////        double sine = Math.sin(robotAngle);
////
////        double frontLeft = r * sine + rightX;// lf
////        double rearLeft = r * cosine + rightX;//lr
////        double rearRight = r * sine - rightX;//rr
////        double frontRight = r * cosine - rightX;//rf
//
//        double frontLeft = 1.0;
//        double rearLeft = 1.0;
//        double rearRight = 1.0;
//        double frontRight = 1.0;
//
//        setMotorPowers(frontLeft, rearLeft, rearRight, frontRight);
//    }

    /**
     * Field Oriented driving
     *
     * @param leftX
     * @param leftY
     * @param rightX
     */
    public void fieldDrive(float leftX, float leftY, float rightX) {
//        The reason we multiply by the sqrt(2) is because the output of sine and cosine when going straight will be sqrt(2)/2. not 1.
//        Multiplying by sqrt(2) will make the straight forward power 1, and while diagonal will be  >1
        double r = Math.hypot(leftX, leftY) * Math.sqrt(2);//Magnitude

//        float angle = imu.getAngle();

//        TThe robot angle is not the angle of the robot is facing, It is the angle in which the robot needs to drive relative to its front,
//        in order to drive at the field-relative angle specified by the driver
//        double robotAngle = Math.atan2(leftY, leftX) - Math.toRadians(Utils.cvtDegrees(angle)) - 3 * (Math.PI / 4);
        double robotAngle = Math.atan2(leftY, leftX) - 3 * (Math.PI / 4);

        double cosine = Math.cos(robotAngle);
        double sine = Math.sin(robotAngle);

        double frontLeft = r * cosine + rightX;
        double rearLeft = r * sine + rightX;
        double rearRight = r * cosine - rightX;
        double frontRight = r * sine - rightX;

        double maxPower = Utils.max(frontLeft, rearLeft, rearRight, frontRight);

        if (maxPower > Constants.MAX_SPEED) {
            frontLeft /= maxPower * (1 / Constants.MAX_SPEED);
            frontRight /= maxPower * (1 / Constants.MAX_SPEED);
            rearLeft /= maxPower * (1 / Constants.MAX_SPEED);
            rearRight /= maxPower * (1 / Constants.MAX_SPEED);
        }

        setMotorPowers(frontLeft, rearLeft, rearRight, frontRight);
    }

    public void turn(int degrees, double sp) {

        double speed = (degrees < 0) ? -1 * sp : sp;

        motorLeftFront.setPower(-speed);
        motorLeftRear.setPower(-speed);
        motorRightFront.setPower(speed);
        motorRightRear.setPower(speed);

        while (imu.getAngle() < degrees && !isStopRequested()) {
            //Ignore
        }
        stopMotors();
    }


    public void print(double inches) {
        LOG.debug("CurrentPosition: " + motorLeftFront.getCurrentPosition() + "" + Constants.inchesToEncoderTicks(inches));
    }

    public void drive(double speed, double leftInches, double rightInches) {
        double newLeftRearTarget;
        double newRightRearTarget;

        double newLeftFrontTarget;
        double newRightFrontTarget;

        // Ensure that the opmode is still active
        if (isOpModeActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = Utils.intToDouble(motorLeftFront.getCurrentPosition()) + Constants.inchesToEncoderTicks(leftInches);
            newLeftRearTarget = Utils.intToDouble(motorLeftRear.getCurrentPosition()) + Constants.inchesToEncoderTicks(leftInches);
            newRightRearTarget = Utils.intToDouble(motorRightRear.getCurrentPosition()) + Constants.inchesToEncoderTicks(rightInches);
            newRightFrontTarget = Utils.intToDouble(motorRightFront.getCurrentPosition()) + Constants.inchesToEncoderTicks(rightInches);

            setTargetTickPosition(motorLeftFront, newLeftFrontTarget);
            setTargetTickPosition(motorLeftRear, newLeftRearTarget);
            setTargetTickPosition(motorRightRear, newRightRearTarget);
            setTargetTickPosition(motorRightFront, newRightFrontTarget);

            // Turn On RUN_TO_POSITION
            setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

            setMotorPowers(speed);
            LOG.debug(String.format("Running to %7f", newLeftFrontTarget));

            float targetAngle = imu.getAngle();

            while (isOpModeActive()
                    && (motorLeftRear.isBusy() && motorRightRear.isBusy())) {

                angleCorrector(speed, targetAngle);
            }

            // Stop all motion;
            stopMotors();
        }
    }

    private void angleCorrector(double speed, float targetAngle) {
        float currentAngle = imu.getAngle();
        float angleDifference = currentAngle - targetAngle;
        if (angleDifference < -180) {
            angleDifference = angleDifference + 360;
        } else if (angleDifference > 180) {
            angleDifference = 360 - angleDifference;
        }

        if (speed > 0D) { //Drive Forward
            if (angleDifference > 1) { //turn right, reduce the speed on right motors
                setMotorPowers(speed, speed, speed - 0.2, speed - 0.2);
            } else if (angleDifference < 1) {//turn left, reduce the speed on left motors
                setMotorPowers(speed - 0.2, speed - 0.2, speed, speed);
            } else {
                setMotorPowers(speed);
            }
        } else {
            if (angleDifference > 1) { //turn right, reduce the speed on left motors
                setMotorPowers(speed - 0.2, speed - 0.2, speed, speed);
            } else if (angleDifference < 1) {//turn left, reduce the speed on right motors
                setMotorPowers(speed, speed, speed - 0.2, speed - 0.2);
            } else {
                setMotorPowers(speed);
            }
        }

    }
}
