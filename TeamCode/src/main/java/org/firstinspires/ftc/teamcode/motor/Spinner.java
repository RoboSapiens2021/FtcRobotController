package org.firstinspires.ftc.teamcode.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Spinner {
    private DcMotorEx motor;

    public Spinner(DcMotorEx dcMotor) {
        this.motor = dcMotor;
    }

    public void init() {
        motor.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void spin(double power) {
        motor.setPower(power);
    }
}
