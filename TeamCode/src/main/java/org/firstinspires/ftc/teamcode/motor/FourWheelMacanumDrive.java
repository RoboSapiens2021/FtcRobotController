package org.firstinspires.ftc.teamcode.motor;

import static org.firstinspires.ftc.teamcode.util.Constants.COLLISION_THRESHOLD_DELTA_G;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
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

    private Acceleration prevAcceleration;

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
        setDirection(motorRightRear, DcMotor.Direction.FORWARD);
        setDirection(motorRightFront, DcMotor.Direction.FORWARD);

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

        prevAcceleration = imu.getLinearAcceleration();
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
    public void driveStraight(double speed, long time) {
        float targetAngle = imu.getAngle();
        long startTime = System.currentTimeMillis();
        boolean collisionDetected = false;

        while (isOpModeActive()
//                && !collisionDetected
                && (System.currentTimeMillis() - startTime) < time) {

            Acceleration currentAcceleration = imu.getLinearAcceleration();

            long timeDiff = System.currentTimeMillis() - startTime;
            double jerkX = abs(currentAcceleration.xAccel - prevAcceleration.xAccel) / timeDiff;
            double jerkY = abs(currentAcceleration.yAccel - prevAcceleration.yAccel) / timeDiff;
            double jerkZ = abs(currentAcceleration.zAccel - prevAcceleration.zAccel) / timeDiff;

            prevAcceleration = currentAcceleration;

            String msg = String.format("jerkX: %.3f jerkY: %.3f jerkZ: %.3f", jerkX, jerkY, jerkZ);
            System.out.println(msg);
            LOG.debug(msg);

            if ((jerkX > COLLISION_THRESHOLD_DELTA_G)
                    || (jerkY > COLLISION_THRESHOLD_DELTA_G)
//                    || (jerkZ > COLLISION_THRESHOLD_DELTA_G)
            ) {
                LOG.warn("collision detected");
                collisionDetected = true;
            }
            angleCorrector(speed,targetAngle);
        }
        stopMotors();
    }

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

            if (speed > 0D) {//Strafing right
                if (angleDifference > 1) {//Strafing right and needs to turn right, reduce speed of back motors
                    //             lFront, lBack,       rBack     , rFront
                    setMotorPowers(speed, -speed + 0.2, speed - 0.2, -speed);//1,-0.8, 0.8,-1
                } else if (angleDifference < -1) {//Strafing right and needs to turn left, reduce speed of front motors
                    setMotorPowers(speed - 0.2, -speed, speed, -speed + 0.2);//0.8,-1,+1, -0.8
                } else {
                    setMotorPowers(speed, -speed, speed, -speed);//1,-1,1,-1
                }
            } else {//Strafing left
                LOG.warn("Code not available");
                //Negative speed made more smaller by adding to it
//                if (angleDifference > 1) {//Strafing left and needs to turn left
//                    setMotorPowers(speed, -speed + 0.2, speed - 0.2, -speed);
//                } else if (angleDifference < -1) {
//                    setMotorPowers(speed - 0.2, -speed, speed, -speed + 0.2);
//                } else {
//                    setMotorPowers(speed, -speed, speed, -speed);
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

//        The robot angle is not the angle of the robot is facing, It is the angle in which the robot needs to drive relative to its front,
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

        while (abs(imu.getAngle()) <= abs(degrees) && !isStopRequested()) {
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
        float currentAngle = imu.getAngle();//IMU angles are positive towards the left
        float angleDifference = currentAngle - targetAngle;
        if (angleDifference < -180) {
            angleDifference = angleDifference + 360;
        } else if (angleDifference > 180) {
            angleDifference = 360 - angleDifference;
        }

//            Correct if the robot is far to the left or to the right
        if (speed > 0D) { //Drive Forward
            if (angleDifference > 1) {//Positive angle hence robot is far to the left and we need to turn to right
//                                 lFront, lBack, rBack     , rFront
                setMotorPowers(speed, speed, speed - 0.2, speed - 0.2);//1,1,0.8,0.8
            } else if (angleDifference < -1) {//turn to left, subtract from left motors rather than the right
                setMotorPowers(speed - 0.2, speed - 0.2, speed, speed);//0.8,0.8,1,1
            } else {
                setMotorPowers(speed);
            }
        } else {//Robot is driving backward
            //Negative speed made more smaller by adding to it
            if (angleDifference > 1) {//Needs to turn right, reduce speed of left motors
                setMotorPowers(speed + 0.2, speed + 0.2, speed, speed);//-1+0.8 , -1+0.2, -1, -1
            } else if (angleDifference < -1) {//Needs to turn left, reduce speed of right motors
                setMotorPowers(speed, speed, speed + 0.2, speed + 0.2);//-1, -1, -1+0.2, -1+0.2
            } else {
                setMotorPowers(speed);
            }
        }

    }
}
