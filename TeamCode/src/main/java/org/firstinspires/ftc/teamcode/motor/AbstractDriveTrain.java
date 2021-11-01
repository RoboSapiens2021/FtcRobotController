package org.firstinspires.ftc.teamcode.motor;

import static org.firstinspires.ftc.teamcode.util.Constants.encoderTicksToInches;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.InertialMotionUnit;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Log;
import org.firstinspires.ftc.teamcode.util.Logger;
import org.firstinspires.ftc.teamcode.util.Utils;

public abstract class AbstractDriveTrain implements DriveTrain {
    private static final Log LOG = Logger.getInstance();

    protected InertialMotionUnit imu;
    protected VoltageSensor batteryVoltageSensor;

    public void setImu(InertialMotionUnit imu) {
        this.imu = imu;
    }

    public void setBatteryVoltageSensor(VoltageSensor batteryVoltageSensor) {
        this.batteryVoltageSensor = batteryVoltageSensor;
    }

    public void setDirection(DcMotorEx motor, DcMotorEx.Direction direction) {
        if (motor != null) {
            motor.setDirection(direction);
        }
    }

    public void setZeroPowerBehavior(DcMotorEx motor, DcMotorEx.ZeroPowerBehavior powerBehavior) {
        if (motor != null) {
            motor.setZeroPowerBehavior(powerBehavior);
        }
    }

    public void setMode(DcMotorEx motor, DcMotorEx.RunMode runMode) {
        if (motor != null) {
            motor.setMode(runMode);
        }
    }

    public void setPIDFCoefficients(DcMotorEx motor, DcMotorEx.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage());

        motor.setPIDFCoefficients(runMode, compensatedCoefficients);
    }

    public void setTargetTickPosition(DcMotorEx motor, double targetPositionInTicks) {
        motor.setTargetPosition(Utils.doubleToInt(targetPositionInTicks));
    }

    @Override
    public void stopMotor(DcMotorEx motor) {
        setMotorPower(motor, Constants.ZERO_POWER);
    }

    public void setMotorPower(DcMotorEx motor, double power) {
        motor.setPower(power);
    }

    public boolean isOpModeActive() {
        return Robot.getInstance().isOpModeActive();
    }

    public boolean isStopRequested() {
        return Robot.getInstance().isStopRequested();
    }

    public double getWheelPosition(DcMotorEx motor) {
        return encoderTicksToInches(motor.getCurrentPosition());
    }

    public double getWheelVelocity(DcMotorEx motor) {
        return encoderTicksToInches(motor.getVelocity());
    }
}
