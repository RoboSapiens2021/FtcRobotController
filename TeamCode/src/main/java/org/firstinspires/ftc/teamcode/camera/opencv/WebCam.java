package org.firstinspires.ftc.teamcode.camera.opencv;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.Logger;
import org.firstinspires.ftc.teamcode.util.Utils;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;

public class WebCam implements OpenCvCamera.AsyncCameraOpenListener, OpenCvCamera.AsyncCameraCloseListener {
    private Logger logger = Logger.getInstance();
    private int cameraMonitorViewId;
    private WebcamName webcamName;

    private boolean useInternalCamera = false;
    private boolean useLivePreview = true;
    private boolean isPreviewPaused = false;

    private CameraResolution cameraResolution;

    private OpenCvCamera camera;
    private OpenCvPipeline pipeline = new LoggingPipeline();
    private volatile boolean initialized = false;
    private volatile boolean initializationError = false;

    public WebCam(int cameraMonitorViewId, WebcamName webcamName) {
        this.cameraMonitorViewId = cameraMonitorViewId;
        this.webcamName = webcamName;
        cameraResolution = new CameraResolution(640, 480);
    }

    public void setUseInternalCamera(boolean useInternalCamera) {
        this.useInternalCamera = useInternalCamera;
    }

    public void setUseLivePreview(boolean useLivePreview) {
        this.useLivePreview = useLivePreview;
    }

    public void setCameraResolution(CameraResolution cameraResolution) {
        this.cameraResolution = cameraResolution;
    }

    public void setPipeline(OpenCvPipeline pipeline) {
        this.pipeline = pipeline;
    }

    public void init() {
        if (useInternalCamera) {
            camera = getInternalCamera(useLivePreview);
        } else {
            camera = getExternalCamera(useLivePreview);
        }
        camera.openCameraDeviceAsync(this);
        camera.setViewportRenderer(cameraResolution.getViewportRenderer());

        while (!initialized && !initializationError) {
            logger.info("Camera waiting for initialization");
            Utils.sleep(2);
        }
    }

    private OpenCvCamera getInternalCamera(boolean livePreviewEnabled) {
        OpenCvCamera camera;
        if (livePreviewEnabled) {
            camera = OpenCvCameraFactory.getInstance().createInternalCamera2(cameraResolution.getCameraDirection(), cameraMonitorViewId);
        } else {
            camera = OpenCvCameraFactory.getInstance().createInternalCamera2(cameraResolution.getCameraDirection());
        }
        camera.setViewportRenderingPolicy(cameraResolution.getLivePreviewRenderingPolicy());
        return camera;
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

    @Override
    public void onOpened() {
        initialized = true;
        initializationError = false;
        logger.info("Camera initialized with pipeline : " + pipeline.getClass().getSimpleName());
        camera.setPipeline(pipeline);
        camera.startStreaming(cameraResolution.getWidth(), cameraResolution.getHeight(), cameraResolution.getRotation());
    }

    @Override
    public void onError(int errorCode) {
        logger.error("Camera error: " + errorCode);
        initialized = false;
        initializationError = true;
        close();
    }

    @Override
    public void onClose() {
        initialized = false;
        if (null != camera) {
            if (useLivePreview) {
                camera.pauseViewport();
            }
//            camera.stopRecordingPipeline();
            camera.stopStreaming();
        }
    }

    public String getStatsAsString() {
        return String.format("FrameCount %d FPS %.2f Total frame time ms %d Pipeline time ms %d Overhead time ms %d Theoretical max FPS %d", camera.getFrameCount(), camera.getFps(), camera.getTotalFrameTimeMs(), camera.getPipelineTimeMs(), camera.getOverheadTimeMs(), camera.getCurrentPipelineMaxFps());
    }

    /**
     * Calling stopStreaming() will stop the stream of images from the camera (and, by extension, stop calling your vision pipeline).
     * HOWEVER, if the reason you wish to stop the stream early is to switch use of the camera over to, say, Vuforia or TFOD, you will also need to call closeCameraDevice()\
     * because according to the Android Camera API documentation: "Your application should only have one Camera object active at a time for a particular hardware camera.
     */
    public void stopStreaming() {
        camera.stopStreaming();
    }

    /**
     * Calling close will internally call stopStreaming()
     */
    public void close() {
        camera.closeCameraDeviceAsync(this);
    }

    /**
     * The primary use case of this is to reduce CPU, memory, and power load
     * when you need your vision pipeline running, but do not require a live preview on the
     * robot controller screen. For instance, this could be useful if you wish to see the live
     * camera preview as you are initializing your robot, but you no longer require the live
     * preview after you have finished your initialization process; pausing the viewport does
     * not stop running your pipeline.
     */
    public void pausePreview() {
        if (useLivePreview) {
            if (isPreviewPaused) {
                camera.resumeViewport();
                isPreviewPaused = false;
            } else {
                camera.pauseViewport();
                isPreviewPaused = true;
            }
        }
    }


}
