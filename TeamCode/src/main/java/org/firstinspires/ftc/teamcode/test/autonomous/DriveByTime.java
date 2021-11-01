package org.firstinspires.ftc.teamcode.test.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.motor.DriveTrain;

@Autonomous(name="DriveByTime", group="test")
public class DriveByTime extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = Robot.getInstance();
        robot.setOpMode(this);
        robot.init();

        DriveTrain driveTrain = robot.configureDriveTrain();
        driveTrain.driveByTime(0.5, 2000);
    }

}

