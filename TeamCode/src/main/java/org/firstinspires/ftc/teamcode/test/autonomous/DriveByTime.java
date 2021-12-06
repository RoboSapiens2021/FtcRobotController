package org.firstinspires.ftc.teamcode.test.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.motor.FourWheelMacanumDrive;

@Autonomous(name = "DriveByTime", group = "test")
public class DriveByTime extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = Robot.getInstance();
        robot.setOpMode(this);
        robot.init();

        FourWheelMacanumDrive driveTrain = robot.getDriveTrain();
        driveTrain.driveByTime(0.5, 2000);
        driveTrain.driveByTime(0.5, 1500);

    }

}

