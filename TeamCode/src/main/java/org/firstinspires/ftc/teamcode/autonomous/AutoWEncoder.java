package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.motor.FourWheelMacanumDrive;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Logger;


@Autonomous(name = "AutoWEncoder", group = "Autonomous")
public class AutoWEncoder extends LinearOpMode {
    private Logger logger = Logger.getInstance();

    @Override
    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();
        Robot robot = Robot.getInstance();
        robot.setOpMode(this);
        robot.init();

        FourWheelMacanumDrive driveTrain = robot.getDriveTrain();
        driveTrain.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        driveTrain.turn(85, 0.5);
//        driveTrain.drive(0.3, 3, 3);
        //robot.getSpinner().spin(0.3);
        //driveTrain.drive(Constants.MAX_SPEED, 5, 5);

        Robot.getInstance().getSpinner().spin((gamepad1.right_trigger - gamepad1.left_trigger));

        // Spin in a circle
        //driveTrain.drive(DRIVE_SPEED,  -33.5,  33.5, 5.0);

    }

}
