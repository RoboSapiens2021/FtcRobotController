package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.motor.DriveTrain;
import org.firstinspires.ftc.teamcode.sensors.QualcommColorSensor;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Logger;


@Autonomous(name = "AutoWEncoder", group = "Autonomous")
public class AutoWEncoder extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = Robot.getInstance();
        robot.setOpMode(this);
        robot.init();

        DriveTrain driveTrain = robot.configureDriveTrain();

        // Go in a square
        driveTrain.drive(Constants.DRIVE_SPEED, 10, 10, 5.0);
        driveTrain.drive(Constants.TURN_SPEED, 5, -5, 4.0);

        driveTrain.drive(Constants.DRIVE_SPEED, 10, 10, 5.0);
        driveTrain.drive(Constants.TURN_SPEED, 5, -5, 4.0);

        driveTrain.drive(Constants.DRIVE_SPEED, 10, 10, 5.0);
        driveTrain.drive(Constants.TURN_SPEED, 5, -5, 4.0);

        driveTrain.drive(Constants.DRIVE_SPEED, 10, 10, 5.0);
        driveTrain.drive(Constants.TURN_SPEED, -5, 5, 4.0);

        driveTrain.drive(Constants.DRIVE_SPEED, 10, 10, 5.0);
        driveTrain.drive(Constants.TURN_SPEED, -5, 5, 4.0);

        driveTrain.drive(Constants.DRIVE_SPEED, 10, 10, 5.0);
        driveTrain.drive(Constants.TURN_SPEED, -5, 5, 4.0);

        driveTrain.drive(Constants.DRIVE_SPEED, 10, 10, 5.0);
        driveTrain.drive(Constants.TURN_SPEED, -5, 5, 4.0);

        driveTrain.drive(Constants.DRIVE_SPEED, 10, 10, 5.0);


        // Spin in a circle
        //driveTrain.drive(DRIVE_SPEED,  -33.5,  33.5, 5.0);

        QualcommColorSensor colorSensor = robot.getQualcommColorSensor();
        Logger logger = Logger.getInstance();
        logger.info("ColorSensor RedValue " + colorSensor.getRedValue());


    }

}
