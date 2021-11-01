package org.firstinspires.ftc.teamcode.sensors;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class ImuAccelerationIntegrator implements BNO055IMU.AccelerationIntegrator {
    private Acceleration acceleration = new Acceleration();

    private BNO055IMU.Parameters parameters;
    private Velocity velocity;
    private Position position;

    @Override
    public void initialize(@NonNull BNO055IMU.Parameters parameters, @Nullable Position initialPosition, @Nullable Velocity initialVelocity) {
        this.parameters = parameters;
        if (null == initialPosition) {
            this.position = new Position();
        } else {
            this.position = initialPosition;
        }
        if (null == initialVelocity) {
            this.velocity = new Velocity();
        } else {
            this.velocity = initialVelocity;
        }
    }

    @Override
    public Position getPosition() {
        return position;
    }

    @Override
    public Velocity getVelocity() {
        return velocity;
    }

    @Override
    public Acceleration getAcceleration() {
        return acceleration;
    }

    @Override
    public void update(Acceleration linearAcceleration) {
        // We should always be given a timestamp here
        if (linearAcceleration.acquisitionTime != 0) {
            Acceleration accelPrev = acceleration;
            this.acceleration = linearAcceleration;
            if (parameters.loggingEnabled) {
                RobotLog.vv(parameters.loggingTag, "dt=%.3fs accel=%s", (acceleration.acquisitionTime - accelPrev.acquisitionTime) * 1e-9, acceleration);
            }
        }
    }

}
