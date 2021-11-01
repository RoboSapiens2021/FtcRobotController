package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.camera.opencv.WebCam;
import org.firstinspires.ftc.teamcode.motor.DriveTrain;
import org.firstinspires.ftc.teamcode.motor.FourWheelMacanumDrive;
import org.firstinspires.ftc.teamcode.motor.Spinner;
import org.firstinspires.ftc.teamcode.sensors.InertialMotionUnit;
import org.firstinspires.ftc.teamcode.sensors.QualcommColorSensor;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Logger;

public class Robot {
    private Logger logger = Logger.getInstance();
    private static final Robot instance = new Robot();

    private LinearOpMode opMode;
    private DriveTrain driveTrain;
    private InertialMotionUnit imu;
    private VoltageSensor batteryVoltageSensor;
    private QualcommColorSensor qualcommColorSensor;
    private Spinner spinner;
    private WebCam webCam;

    private Robot() {
    }

    public static Robot getInstance() {
        return instance;
    }

    public void setOpMode(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    /**
     * Initialize the system
     */
    public void init() {
        logger.setTelemetry(opMode.telemetry);
        HardwareMap hardwareMap = opMode.hardwareMap;
//
//        batteryVoltageSensor = opMode.hardwareMap.voltageSensor.iterator().next();

        this.imu = configureIMU(hardwareMap);

        FourWheelMacanumDrive fourWheelMacanumDrive = configureDriveTrain(hardwareMap);
//        fourWheelMacanumDrive.setBatteryVoltageSensor(batteryVoltageSensor);
        fourWheelMacanumDrive.setImu(imu);
        this.driveTrain = fourWheelMacanumDrive;


//        ColorSensor colorSensor = hardwareMap.colorSensor.get(Constants.DEVICE_NAME_COLOR_SENSOR);
//        qualcommColorSensor = new QualcommColorSensor(colorSensor);
//        qualcommColorSensor.init();

        spinner = new Spinner(hardwareMap.get(DcMotorEx.class, Constants.WHEEL_NAME.SPINNER.name()));
        spinner.init();

//        this.webCam = configureWebCam(hardwareMap);

        opMode.waitForStart();
    }

    private InertialMotionUnit configureIMU(HardwareMap hardwareMap) {
        logger.debug("Initializing IMU");
        BNO055IMU bno055IMU = hardwareMap.get(BNO055IMU.class, Constants.DEVICE_NAME.imu.name());
        InertialMotionUnit imu = new InertialMotionUnit(bno055IMU);
        imu.init();
        logger.info(imu.getStatus());
        logger.debug("Initialized IMU");

        return imu;
    }

    private FourWheelMacanumDrive configureDriveTrain(HardwareMap hardwareMap) {
        logger.debug("Initializing Drive");
        DcMotorEx motorLeftFront = hardwareMap.get(DcMotorEx.class, Constants.WHEEL_NAME.LEFT_FRONT.name());
        DcMotorEx motorLeftRear = hardwareMap.get(DcMotorEx.class, Constants.WHEEL_NAME.LEFT_REAR.name());
        DcMotorEx motorRightRear = hardwareMap.get(DcMotorEx.class, Constants.WHEEL_NAME.RIGHT_REAR.name());
        DcMotorEx motorRightFront = hardwareMap.get(DcMotorEx.class, Constants.WHEEL_NAME.RIGHT_FRONT.name());

        FourWheelMacanumDrive fourWheelMacanumDrive = new FourWheelMacanumDrive(motorLeftFront, motorLeftRear, motorRightRear, motorRightFront);
        fourWheelMacanumDrive.init();
        fourWheelMacanumDrive.reset();

        logger.debug("Initialized Drive");
        return fourWheelMacanumDrive;
    }

    private WebCam configureWebCam(HardwareMap hardwareMap) {
        logger.debug("Initializing Drive");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, Constants.DEVICE_NAME.WEBCAM.name());
        WebCam webCam = new WebCam(cameraMonitorViewId, webcamName);
        webCam.setPipeline(Constants.pipeline);
        webCam.init();

        logger.debug("Initialized WebCam");
        return webCam;
    }


    public DriveTrain configureDriveTrain() {
        return driveTrain;
    }

    public InertialMotionUnit getImu() {
        return imu;
    }

    public boolean isStopRequested() {
        return opMode.isStopRequested();
    }

    public boolean isOpModeActive() {
        return opMode.opModeIsActive();
    }

    public QualcommColorSensor getQualcommColorSensor() {
        return qualcommColorSensor;
    }

    public Spinner getSpinner() {
        return spinner;
    }

    public WebCam getWebCam() {
        return webCam;
    }
}
