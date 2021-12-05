package org.firstinspires.ftc.teamcode.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;

public class Spinner {
    private DcMotorEx motor;

    public Spinner(DcMotorEx dcMotor) {
        this.motor = dcMotor;
    }

    public void init() {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void spin(double power) {
        motor.setPower(power);
    }

    public void spin(double power, long time) {
        long startTime = System.currentTimeMillis();

        while (Robot.getInstance().isOpModeActive() && (System.currentTimeMillis() - startTime) < time) {
            motor.setPower(power);
        }
        motor.setPower(Constants.ZERO_POWER);
    }
}
