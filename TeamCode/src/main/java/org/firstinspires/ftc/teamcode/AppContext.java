package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.drivetrain.FourWheelMacanumDrive;
import org.firstinspires.ftc.teamcode.drivetrain.Imu;
import org.firstinspires.ftc.teamcode.util.DriveConstants;
import org.firstinspires.ftc.teamcode.util.Logger;

public class AppContext {
    private static final AppContext instance = new AppContext();

    private LinearOpMode opMode;
    private DriveTrain driveTrain;
    private Imu imu;
    private VoltageSensor batteryVoltageSensor;

    private AppContext() {
    }

    public static AppContext getInstance() {
        return instance;
    }

    public void setOpMode(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init() {
        Logger logger = Logger.getInstance();
        logger.setTelemetry(opMode.telemetry);
//
//        batteryVoltageSensor = opMode.hardwareMap.voltageSensor.iterator().next();
//
//        //IMU Initialization
//        imu = new Imu(opMode.hardwareMap);
//        imu.init();

        DcMotorEx motorLeftFront = opMode.hardwareMap.get(DcMotorEx.class, DriveConstants.WHEEL_NAME.LEFT_FRONT.name());
        DcMotorEx motorLeftRear = opMode.hardwareMap.get(DcMotorEx.class, DriveConstants.WHEEL_NAME.LEFT_REAR.name());
        DcMotorEx motorRightRear = opMode.hardwareMap.get(DcMotorEx.class, DriveConstants.WHEEL_NAME.RIGHT_REAR.name());
        DcMotorEx motorRightFront = opMode.hardwareMap.get(DcMotorEx.class, DriveConstants.WHEEL_NAME.RIGHT_FRONT.name());

        FourWheelMacanumDrive fourWheelMacanumDrive = new FourWheelMacanumDrive(motorLeftFront, motorLeftRear, motorRightRear, motorRightFront);
        fourWheelMacanumDrive.setBatteryVoltageSensor(batteryVoltageSensor);
        fourWheelMacanumDrive.setImu(imu);

        fourWheelMacanumDrive.init();
//        fourWheelMacanumDrive.reset();

        this.driveTrain = fourWheelMacanumDrive;

        opMode.waitForStart();
    }

    public DriveTrain getDriveTrain() {
        return driveTrain;
    }

    public Imu getImu() {
        return imu;
    }

    public boolean isStopRequested() {
        return opMode.isStopRequested();
    }

    public boolean isOpModeActive() {
        return opMode.opModeIsActive();
    }
}
