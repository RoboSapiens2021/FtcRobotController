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

@Autonomous(name = "AutoWithEncoder", group = "Autonomous")
public class AutoWithEncoder extends LinearOpMode {
    private Logger logger = Logger.getInstance();
    private static final double SPIN_SPEED = 0.3;
    private ExecutorService threadPool = Executors.newSingleThreadExecutor();
    private PositionSelectionThread positionSelectionThread = new PositionSelectionThread();

    private BitSet bitSet = new BitSet(4);

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

        int angle = 90;
        int spin_time = 3500;
        int warehouse_run_time = 1800;
        int front_back_run_time = 700;
        int warehouse_back_run_time = 1400;

        if (bitSet.get(0)) {//Blue Front - X
//            driveTrain.turn(angle, 0.3);
            driveTrain.driveStraight(-0.7, front_back_run_time);

            spinner.spin(SPIN_SPEED, spin_time);
            driveTrain.driveStraight(Constants.MAX_SPEED, warehouse_run_time);
        } else if (bitSet.get(1)) {//Red Front - B
//            driveTrain.turn(-angle, 0.3);
            driveTrain.driveStraight(-0.7, front_back_run_time);

            spinner.spin(-SPIN_SPEED, spin_time);
            driveTrain.driveStraight(Constants.MAX_SPEED, warehouse_run_time);
        } else if (bitSet.get(2)) { //Red Warehouse - A
//            driveTrain.turn(-angle, 0.3);
            driveTrain.driveStraight(-0.7, warehouse_back_run_time);

            spinner.spin(-SPIN_SPEED, spin_time);
            driveTrain.driveStraight(Constants.MAX_SPEED, warehouse_run_time);
        } else {//Blue Warehouse - Y
//            driveTrain.turn(angle, 0.3);
            driveTrain.driveStraight(-0.7, warehouse_back_run_time);

            spinner.spin(SPIN_SPEED, spin_time);
            driveTrain.driveStraight(Constants.MAX_SPEED, warehouse_run_time);
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
            telemetry.addLine("Select X for Blue Front");
            telemetry.addLine("Select B for Red Front");
            telemetry.addLine("Select A for Red Warehouse");
            telemetry.addLine("Select Y for Blue Warehouse");
            telemetry.update();

            while (!stop) {
                if (gamepad1.x) {
                    logger.debug("Robot start position: Blue Front");
                    bitSet.clear();
                    bitSet.set(0);
                } else if (gamepad1.b) {
                    logger.debug("Robot start position: Red Front");
                    bitSet.clear();
                    bitSet.set(1);
                } else if (gamepad1.a) {
                    logger.debug("Robot start position: Red Warehouse");
                    bitSet.clear();
                    bitSet.set(2);
                } else if (gamepad1.y) {
                    logger.debug("Robot start position: Blue Warehouse");
                    bitSet.clear();
                    bitSet.set(3);
                }
            }
        }

    }

}


