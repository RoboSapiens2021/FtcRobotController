package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drivetrain.DriveTrain;

@Autonomous(name = "ImuExample")

public class ImuExample extends LinearOpMode {

    @Override
    public void runOpMode() {
        AppContext appContext = AppContext.getInstance();
        appContext.setOpMode(this);
        appContext.init();

        DriveTrain driveTrain = appContext.getDriveTrain();

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


