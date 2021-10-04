package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AppContext;
import org.firstinspires.ftc.teamcode.drivetrain.DriveTrain;

/*
 * This is a simple routine to test turning capabilities.
 */
@Autonomous(group = "test")
public class TurnTest extends LinearOpMode {
    public static int ANGLE = 90; // deg

    @Override
    public void runOpMode() {
        AppContext appContext = AppContext.getInstance();
        appContext.setOpMode(this);
        appContext.init();

        DriveTrain drive = appContext.getDriveTrain();

        if (isStopRequested()) {
            return;
        }

        drive.turn(ANGLE);
    }
}
