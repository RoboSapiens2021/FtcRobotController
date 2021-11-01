package org.firstinspires.ftc.teamcode.test.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Utils;

@Autonomous(name = "MotorConnectionTest", group = "test")
public class MotorConnectionTest extends LinearOpMode {
    public static double MOTOR_POWER = 0.7;

    @Override
    public void runOpMode() {
        DcMotorEx motorLeftFront = hardwareMap.get(DcMotorEx.class, Constants.WHEEL_NAME.LEFT_FRONT.name());
        DcMotorEx motorLeftRear = hardwareMap.get(DcMotorEx.class, Constants.WHEEL_NAME.LEFT_REAR.name());
        DcMotorEx motorRightRear = hardwareMap.get(DcMotorEx.class, Constants.WHEEL_NAME.RIGHT_REAR.name());
        DcMotorEx motorRightFront = hardwareMap.get(DcMotorEx.class, Constants.WHEEL_NAME.RIGHT_FRONT.name());

//        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
//        motorLeftRear.setDirection(DcMotor.Direction.REVERSE);
//        motorRightRear.setDirection(DcMotor.Direction.REVERSE);
        motorRightFront.setDirection(DcMotor.Direction.REVERSE);

        motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (isStopRequested()) {
            return;
        }

        while (!isStopRequested()) {
            motorLeftFront.setPower(MOTOR_POWER);
            Utils.sleep(2000);
            motorLeftFront.setPower(0);

            motorLeftRear.setPower(MOTOR_POWER);
            Utils.sleep(2000);
            motorLeftRear.setPower(0);

            motorRightRear.setDirection(DcMotor.Direction.REVERSE);
            motorRightRear.setPower(MOTOR_POWER);
            Utils.sleep(2000);
            motorRightRear.setPower(0);

//            motorRightFront.setDirection(DcMotor.Direction.REVERSE);
            motorRightFront.setPower(MOTOR_POWER);
            Utils.sleep(2000);
            motorRightFront.setPower(0);
        }
    }
}
