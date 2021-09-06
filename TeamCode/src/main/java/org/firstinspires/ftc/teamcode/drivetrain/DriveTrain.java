package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public interface DriveTrain {
    void init();

    void reset();

    void setDirection(DcMotorEx motor, DcMotorEx.Direction direction);

    void setZeroPowerBehavior(DcMotorEx motor, DcMotorEx.ZeroPowerBehavior powerBehavior);

    void setMode(DcMotorEx motor, DcMotorEx.RunMode runMode);

    void stopMotor(DcMotorEx motor);

    void setMotorPowers(double v1, double v2, double v3, double v4);

    void setTargetTickPosition(DcMotorEx motor, double targetPosition);

    void driveByTime(double power, long time);

    void driveStraight(double power, long time);

    void drive(double speed, double leftInches, double rightInches, double timeout);

    void turn(int degrees);

}
