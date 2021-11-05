package org.firstinspires.ftc.teamcode.test.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.motor.FourWheelMacanumDrive;

@Autonomous(name = "ImuExample", group = "test")
public class ImuExample extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = Robot.getInstance();
        robot.setOpMode(this);
        robot.init();

        FourWheelMacanumDrive driveTrain = robot.getDriveTrain();

        driveTrain.print(1);

        driveTrain.drive(1, 2, 2);
        driveTrain.print(1);
        driveTrain.turn(92, 0.8);


    }


}


