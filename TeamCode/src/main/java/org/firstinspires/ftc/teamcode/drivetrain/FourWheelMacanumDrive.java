package org.firstinspires.ftc.teamcode.drivetrain;

import static org.firstinspires.ftc.teamcode.util.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.util.DriveConstants.RUN_USING_ENCODER;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.DriveConstants;
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

        setDirection(motorRightRear, DcMotor.Direction.REVERSE);
        setDirection(motorRightFront, DcMotor.Direction.REVERSE);

        for (DcMotorEx motor : motors) {
//            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
//            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
//            motor.setMotorType(motorConfigurationType);

            setZeroPowerBehavior(motor, DcMotor.ZeroPowerBehavior.BRAKE);

            if (RUN_USING_ENCODER) {
                setMode(motor, DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
                setPIDFCoefficients(motor, DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
            }

        }

    }

    public void reset() {
        LOG.log(Logger.CAPTION.Status, "Resetting Wheel Encoders");

        for (DcMotorEx motor : motors) {
            setMode(motor, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        logPosition();
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

    @Override
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

    private void setMotorMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            setMode(motor, runMode);
        }
    }

    public void driveByTime(double power, long time) {
        setMotorPowers(power);

        Utils.sleep(time);

        setMotorPowers(DriveConstants.ZERO_POWER);
    }

    public void driveStraight(double power, long time) {
        driveByTime(power, time);
    }

    public void turn(int degrees) {
        float angularOrientation = imu.getAngularOrientation();
        LOG.log(Log.CAPTION.Position, angularOrientation);

        Utils.sleep(500);

        motorLeftFront.setPower(-0.2);
        motorLeftRear.setPower(-0.2);

        motorRightFront.setPower(0.2);
        motorRightRear.setPower(0.2);

        while (angularOrientation < degrees && !isStopRequested()) {
            angularOrientation = imu.getAngularOrientation();
            LOG.log(Log.CAPTION.Position, angularOrientation);
        }
        stopMotors();
    }

    public void drive(double speed, double leftInches, double rightInches, double timeout) {
        double newLeftRearTarget;
        double newRightRearTarget;

        double newLeftFrontTarget;
        double newRightFrontTarget;

        ElapsedTime runtime = new ElapsedTime();

        // Ensure that the opmode is still active
        if (isOpModeActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = Utils.intToDouble(motorLeftFront.getCurrentPosition()) + DriveConstants.inchesToEncoderTicks(leftInches);
            newLeftRearTarget = Utils.intToDouble(motorLeftRear.getCurrentPosition()) + DriveConstants.inchesToEncoderTicks(leftInches);
            newRightRearTarget = Utils.intToDouble(motorRightRear.getCurrentPosition()) + DriveConstants.inchesToEncoderTicks(rightInches);
            newRightFrontTarget = Utils.intToDouble(motorRightFront.getCurrentPosition()) + DriveConstants.inchesToEncoderTicks(rightInches);

            setTargetTickPosition(motorLeftFront, newLeftFrontTarget);
            setTargetTickPosition(motorLeftRear, newLeftRearTarget);
            setTargetTickPosition(motorRightRear, newRightRearTarget);
            setTargetTickPosition(motorRightFront, newRightFrontTarget);

            // Turn On RUN_TO_POSITION
            setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            setMotorPowers(speed);
            LOG.log(Logger.CAPTION.Position, "Running to %7d", newLeftFrontTarget);

            while (isOpModeActive()
                    && (runtime.seconds() < timeout)
                    && (motorLeftRear.isBusy() && motorRightRear.isBusy())) {

                // Display it for the driver.
                LOG.log(Logger.CAPTION.Position, "Running to %7d", motorLeftFront.getCurrentPosition());
            }

            // Stop all motion;
            stopMotors();

            // Turn off RUN_TO_POSITION
            setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void logPosition() {
        LOG.log(Logger.CAPTION.Position, "Starting at %7d :%7d", motorLeftRear.getCurrentPosition(), motorRightRear.getCurrentPosition());
    }
}
