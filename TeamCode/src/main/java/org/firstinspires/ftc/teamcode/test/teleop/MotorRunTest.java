package org.firstinspires.ftc.teamcode.test.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Constants;

@TeleOp(name = "MotorRunTest", group = "test")
public class MotorRunTest extends LinearOpMode {
    public static double MOTOR_POWER = 0.1;

    @Override
    public void runOpMode() {
        DcMotorEx motorLeftFront = hardwareMap.get(DcMotorEx.class, Constants.WHEEL_NAME.LEFT_FRONT.name());
        DcMotorEx motorLeftRear = hardwareMap.get(DcMotorEx.class, Constants.WHEEL_NAME.LEFT_REAR.name());
        DcMotorEx motorRightRear = hardwareMap.get(DcMotorEx.class, Constants.WHEEL_NAME.RIGHT_REAR.name());
        DcMotorEx motorRightFront = hardwareMap.get(DcMotorEx.class, Constants.WHEEL_NAME.RIGHT_FRONT.name());

        motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while (!isStopRequested()) {
            motorLeftFront.setPower(MOTOR_POWER);
            motorLeftRear.setPower(MOTOR_POWER);
            motorRightRear.setPower(MOTOR_POWER);
            motorRightFront.setPower(MOTOR_POWER);
        }

    }
}
