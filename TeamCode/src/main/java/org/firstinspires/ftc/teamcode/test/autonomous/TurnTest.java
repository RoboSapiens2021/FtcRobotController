package org.firstinspires.ftc.teamcode.test.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.motor.FourWheelMacanumDrive;

/*
 * This is a simple routine to test turning capabilities.
 */
@Autonomous(name = "TurnTest", group = "test")
public class TurnTest extends LinearOpMode {
    public static int ANGLE = 90; // deg

    @Override
    public void runOpMode() {
        Robot robot = Robot.getInstance();
        robot.setOpMode(this);
        robot.init();

        FourWheelMacanumDrive driveTrain = robot.getDriveTrain();

        if (isStopRequested()) {
            return;
        }

        driveTrain.turn(ANGLE,0.8);
    }
}
