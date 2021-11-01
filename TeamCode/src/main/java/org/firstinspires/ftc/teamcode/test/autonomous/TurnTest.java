package org.firstinspires.ftc.teamcode.test.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.motor.DriveTrain;

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

        DriveTrain drive = robot.configureDriveTrain();

        if (isStopRequested()) {
            return;
        }

        drive.turn(ANGLE);
    }
}
