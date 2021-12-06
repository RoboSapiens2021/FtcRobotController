package org.firstinspires.ftc.teamcode.test.camera;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.camera.opencv.CameraResolution;
import org.firstinspires.ftc.teamcode.camera.opencv.ShapeDetectionPipeline;
import org.firstinspires.ftc.teamcode.camera.opencv.SkystoneDeterminationPipeline;
import org.firstinspires.ftc.teamcode.camera.opencv.StageSwitchingPipeline;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Logger;
import org.firstinspires.ftc.teamcode.util.RoboSwitches;
import org.firstinspires.ftc.teamcode.util.Utils;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp(name = "CameraTest", group = "test")
public class CameraTest extends LinearOpMode {
    private static Logger logger = Logger.getInstance();

    private int cameraMonitorViewId;
    private WebcamName webcamName;
    //    private CameraResolution cameraResolution = new CameraResolution(640, 480);
//    private CameraResolution cameraResolution = new CameraResolution(1280, 720);
//    private CameraResolution cameraResolution = new CameraResolution(1920, 1080);
    private CameraResolution cameraResolution = new CameraResolution(320, 240);

    //    public static final OpenCvPipeline pipeline = new StageSwitchingPipeline();
//    public static final OpenCvPipeline pipeline = new BoxAroundPipeline();
//    public static final OpenCvPipeline pipeline = new SkystoneDeterminationPipeline();
//    public static final OpenCvPipeline pipeline = new LoggingPipeline();
    public static final OpenCvPipeline pipeline = new ShapeDetectionPipeline();

    @Override
    public void runOpMode() throws InterruptedException {
        logger.setTelemetry(telemetry);

        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        logger.debug("cameraMonitorViewId: " + cameraMonitorViewId);
        webcamName = hardwareMap.get(WebcamName.class, Constants.DEVICE_NAME.WEBCAM.name());
        logger.debug("webcamName: " + webcamName);

        CameraTestListener cameraListener = new CameraTestListener();
        OpenCvCamera camera = getExternalCamera(RoboSwitches.CAMERA_LIVE_PREVIEW);

        camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
        camera.openCameraDeviceAsync(cameraListener);
        while (!cameraListener.isInitialized()) {
            logger.info("Camera waiting for initialization");
        }

        logger.info("Camera initialized with pipeline : " + pipeline.getClass().getSimpleName());
        camera.setPipeline(pipeline);
        camera.startStreaming(cameraResolution.getWidth(), cameraResolution.getHeight(), cameraResolution.getRotation());
        logger.debug(String.format("Streaming at %dX%d resolution", cameraResolution.getWidth(), cameraResolution.getHeight()));

        waitForStart();

        try {
            while (opModeIsActive()) {
                if (pipeline instanceof StageSwitchingPipeline) {
                    logger.debug("NumContoursFound: " + ((StageSwitchingPipeline) pipeline).getNumContoursFound());
                } else if (pipeline instanceof SkystoneDeterminationPipeline) {
                    logger.debug("Analysis: " + ((SkystoneDeterminationPipeline) pipeline).getAnalysis());
                }

//                else if (pipeline instanceof ShapeDetectionPipeline) {
//                    logger.debug("Analysis: " + ((ShapeDetectionPipeline) pipeline).);
//                } else {
//                    logger.debug(String.format("FrameCount %d FPS %.2f Total frame time ms %d Pipeline time ms %d Overhead time ms %d Theoretical max FPS %d", camera.getFrameCount(),
//                            camera.getFps(), camera.getTotalFrameTimeMs(), camera.getPipelineTimeMs(), camera.getOverheadTimeMs(), camera.getCurrentPipelineMaxFps()));
//                }

                Utils.sleep(50);
            }
        } finally {
            if (null != camera) {
                camera.closeCameraDevice();
            }
        }
    }

    private OpenCvCamera getExternalCamera(boolean livePreviewEnabled) {
        OpenCvCamera camera;
        if (livePreviewEnabled) {
            camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        } else {
            camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        }
        return camera;
    }

    private class CameraTestListener implements OpenCvCamera.AsyncCameraOpenListener {
        private volatile boolean initialized = false;

        public boolean isInitialized() {
            return initialized;
        }

        @Override
        public void onOpened() {
            initialized = true;
        }

        @Override
        public void onError(int errorCode) {
            logger.error("Camera error: " + errorCode);
        }

    }

}
