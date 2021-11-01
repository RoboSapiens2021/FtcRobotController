package org.firstinspires.ftc.teamcode.test.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.motor.DriveTrain;

@Autonomous(name = "ImuExample", group = "test")
public class ImuExample extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = Robot.getInstance();
        robot.setOpMode(this);
        robot.init();

        DriveTrain driveTrain = robot.configureDriveTrain();

        driveTrain.driveStraight(1, 500);
        driveTrain.turn(92);
        driveTrain.driveStraight(1, 1100);
        driveTrain.turn(0);
        driveTrain.driveStraight(1, 1100);
        driveTrain.turn(-25);
        driveTrain.driveStraight(1, 1200);

        driveTrain.turn(0);
        driveTrain.driveStraight(-1, 1864);
        driveTrain.turn(-90);
        driveTrain.driveStraight(1, 1100);
        driveTrain.turn(0);
        driveTrain.driveStraight(1, 1100);
        driveTrain.turn(25);
        driveTrain.driveStraight(1, 1200);

    }


}


