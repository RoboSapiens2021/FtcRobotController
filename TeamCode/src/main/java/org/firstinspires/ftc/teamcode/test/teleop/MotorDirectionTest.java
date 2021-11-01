package org.firstinspires.ftc.teamcode.test.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Constants;

/**
 * This is a simple teleop routine for debugging your motor configuration.
 * Pressing each of the buttons will power its respective motor.
 * <p>
 * Button Mappings:
 * <p>
 * Xbox/PS4 Button - Motor
 * X / ▢         - Front Left
 * Y / Δ         - Front Right
 * B / O         - Rear  Right
 * A / X         - Rear  Left
 * The buttons are mapped to match the wheels spatially if you
 * were to rotate the gamepad 45deg°. x/square is the front left
 * ________        and each button corresponds to the wheel as you go clockwise
 * / ______ \
 * ------------.-'   _  '-..+              Front of Bot
 * /   _  ( Y )  _  \                  ^
 * |  ( X )  _  ( B ) |     Front Left   \    Front Right
 * ___  '.      ( A )     /|       Wheel       \      Wheel
 * .'    '.    '-._____.-'  .'       (x/▢)        \     (Y/Δ)
 * |       |                 |                      \
 * '.___.' '.               |          Rear Left    \   Rear Right
 * '.             /             Wheel       \    Wheel
 * \.          .'              (A/X)        \   (B/O)
 * \________/
 * <p>
 * Uncomment the @Disabled tag below to use this opmode.
 */
@TeleOp(name = "MotorDirectionTest", group = "test")
@Disabled
public class MotorDirectionTest extends LinearOpMode {
    public static double MOTOR_POWER = 0.7;

    @Override
    public void runOpMode() {
        DcMotorEx motorLeftFront = hardwareMap.get(DcMotorEx.class, Constants.WHEEL_NAME.LEFT_FRONT.name());
        DcMotorEx motorLeftRear = hardwareMap.get(DcMotorEx.class, Constants.WHEEL_NAME.LEFT_REAR.name());
        DcMotorEx motorRightRear = hardwareMap.get(DcMotorEx.class, Constants.WHEEL_NAME.RIGHT_REAR.name());
        DcMotorEx motorRightFront = hardwareMap.get(DcMotorEx.class, Constants.WHEEL_NAME.RIGHT_FRONT.name());

        motorRightRear.setDirection(DcMotor.Direction.REVERSE);

        motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Press play to begin the debugging opmode");
        telemetry.update();

        if (isStopRequested()) {
            return;
        }

        telemetry.clearAll();
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        while (!isStopRequested()) {
            telemetry.addLine("Press each button to turn on its respective motor");
            telemetry.addLine();
            telemetry.addLine("<font face=\"monospace\">Xbox/PS4 Button - Motor</font>");
            telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;X / ▢&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Front Left</font>");
            telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;Y / Δ&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Front Right</font>");
            telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;B / O&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Rear&nbsp;&nbsp;Right</font>");
            telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;A / X&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Rear&nbsp;&nbsp;Left</font>");
            telemetry.addLine();

            if (gamepad1.x) {
                motorLeftFront.setPower(MOTOR_POWER);
                telemetry.addLine("Running Motor: Front Left");
            } else if (gamepad1.y) {
                motorRightFront.setPower(MOTOR_POWER);
                telemetry.addLine("Running Motor: Front Right");
            } else if (gamepad1.b) {
                motorRightRear.setPower(MOTOR_POWER);
                telemetry.addLine("Running Motor: Rear Right");
            } else if (gamepad1.a) {
                motorLeftRear.setPower(MOTOR_POWER);
                telemetry.addLine("Running Motor: Rear Left");
            } else {
//                motorLeftFront.setPower(MOTOR_POWER);
//                motorLeftRear.setPower(MOTOR_POWER);
//                motorRightRear.setPower(MOTOR_POWER);
//                motorRightFront.setPower(MOTOR_POWER);
                telemetry.addLine("Running Motor: None");
            }

            telemetry.update();
        }
    }
}
