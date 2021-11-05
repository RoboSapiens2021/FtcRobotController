package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.RoboSwitches.RUN_USING_COLOR_SENSOR;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.camera.opencv.WebCam;
import org.firstinspires.ftc.teamcode.motor.FourWheelMacanumDrive;
import org.firstinspires.ftc.teamcode.motor.Spinner;
import org.firstinspires.ftc.teamcode.sensors.HSVColorSensor;
import org.firstinspires.ftc.teamcode.sensors.InertialMotionUnit;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Logger;

public class Robot {
    private Logger logger = Logger.getInstance();
    private static final Robot instance = new Robot();

    private LinearOpMode opMode;
    private FourWheelMacanumDrive driveTrain;
    private InertialMotionUnit imu;
    private VoltageSensor batteryVoltageSensor;
    private HSVColorSensor hsvColorSensor;
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

        driveTrain = configureDriveTrain(hardwareMap);
//        driveTrain.setBatteryVoltageSensor(batteryVoltageSensor);
        driveTrain.setImu(imu);

        hsvColorSensor = configureColorSensor(hardwareMap);

        spinner = new Spinner(hardwareMap.get(DcMotorEx.class, Constants.WHEEL_NAME.SPINNER.name()));
        spinner.init();

//        this.webCam = configureWebCam(hardwareMap);

        opMode.waitForStart();
    }

    private HSVColorSensor configureColorSensor(HardwareMap hardwareMap) {
        NormalizedColorSensor colorSensor;
        if (RUN_USING_COLOR_SENSOR) {
            colorSensor = hardwareMap.get(NormalizedColorSensor.class, Constants.DEVICE_NAME.COLOR_SENSOR.name());

            HSVColorSensor hsvColorSensor = new HSVColorSensor(colorSensor);
            hsvColorSensor.setGain(Constants.COLOR_SENSOR_GAIN);
            hsvColorSensor.init();
        }

        return hsvColorSensor;
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

        DcMotorEx motorLeftFront = hardwareMap.get(DcMotorEx.class, Constants.WHEEL_NAME.LEFT_FRONT.name());
        DcMotorEx motorLeftRear = hardwareMap.get(DcMotorEx.class, Constants.WHEEL_NAME.LEFT_REAR.name());
        DcMotorEx motorRightRear = hardwareMap.get(DcMotorEx.class, Constants.WHEEL_NAME.RIGHT_REAR.name());
        DcMotorEx motorRightFront = hardwareMap.get(DcMotorEx.class, Constants.WHEEL_NAME.RIGHT_FRONT.name());

        FourWheelMacanumDrive fourWheelMacanumDrive = new FourWheelMacanumDrive(motorLeftFront, motorLeftRear, motorRightRear, motorRightFront);
        fourWheelMacanumDrive.init();

        return fourWheelMacanumDrive;
    }


    private WebCam configureWebCam(HardwareMap hardwareMap) {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, Constants.DEVICE_NAME.WEBCAM.name());
        WebCam webCam = new WebCam(cameraMonitorViewId, webcamName);
        webCam.setPipeline(Constants.pipeline);
        webCam.init();


        return webCam;
    }

    public FourWheelMacanumDrive getDriveTrain() {
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

    public HSVColorSensor getHsvColorSensor() {
        return hsvColorSensor;
    }

    public Spinner getSpinner() {
        return spinner;
    }

    public WebCam getWebCam() {
        return webCam;
    }
}
