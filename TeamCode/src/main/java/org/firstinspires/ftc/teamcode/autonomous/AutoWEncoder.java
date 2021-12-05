package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.motor.FourWheelMacanumDrive;
import org.firstinspires.ftc.teamcode.motor.Spinner;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Logger;

import java.util.BitSet;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;


@Autonomous(name = "AutoWEncoder", group = "Autonomous")
public class AutoWEncoder extends LinearOpMode {
    private Logger logger = Logger.getInstance();
    private static final double SPIN_SPEED = 0.4;
    private ExecutorService threadPool = Executors.newSingleThreadExecutor();
    private PositionSelectionThread positionSelectionThread = new PositionSelectionThread();

    private BitSet bitSet = new BitSet(2);

    @Override
    public void runOpMode() {
        Robot robot = Robot.getInstance();
        robot.setOpMode(this);
        bitSet.set(0);//Front
        threadPool.submit(positionSelectionThread);
        robot.init();

        Spinner spinner = Robot.getInstance().getSpinner();
        FourWheelMacanumDrive driveTrain = robot.getDriveTrain();
        driveTrain.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (bitSet.get(0)) {//Front - X
            driveTrain.driveStraight(-0.8, 560);

            spinner.spin(SPIN_SPEED, 2000);
            driveTrain.driveStraight(Constants.MAX_SPEED, 2000);
        } else if (bitSet.get(1)) { //Warehouse - B
            driveTrain.driveStraight(-0.8, 1200);
            spinner.spin(SPIN_SPEED, 2000);
            driveTrain.driveStraight(Constants.MAX_SPEED, 2400);
        }

        positionSelectionThread.stop();
        threadPool.shutdownNow();
    }

    private class PositionSelectionThread implements Runnable {
        private volatile boolean stop = false;

        public void stop() {
            this.stop = true;
        }

        public void run() {
            telemetry.addLine("Select X for Front (Default)");
            telemetry.addLine("Select B for Warehouse");
            telemetry.update();

            while (!stop) {
                if (gamepad1.x) {
                    logger.debug("Robot start position: Front");
                    bitSet.clear();
                    bitSet.set(0);
                } else if (gamepad1.b) {
                    logger.debug("Robot start position: Warehouse");
                    bitSet.clear();
                    bitSet.set(1);
                }
            }
        }

    }

}


