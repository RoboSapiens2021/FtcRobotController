package org.firstinspires.ftc.teamcode.camera.opencv;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;

public class CameraResolution {
    private int width;
    private int height;
    private OpenCvCameraRotation rotation = OpenCvCameraRotation.UPRIGHT;
    private OpenCvInternalCamera2.CameraDirection cameraDirection = OpenCvInternalCamera2.CameraDirection.BACK;
    private OpenCvCamera.ViewportRenderingPolicy livePreviewRenderingPolicy = OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW;
    private OpenCvCamera.ViewportRenderer viewportRenderer = OpenCvCamera.ViewportRenderer.GPU_ACCELERATED;

    public CameraResolution(int width, int height) {
        this.width = width;
        this.height = height;
    }

    public int getWidth() {
        return width;
    }

    public void setWidth(int width) {
        this.width = width;
    }

    public int getHeight() {
        return height;
    }

    public void setHeight(int height) {
        this.height = height;
    }

    public OpenCvCameraRotation getRotation() {
        return rotation;
    }

    public void setRotation(OpenCvCameraRotation rotation) {
        this.rotation = rotation;
    }

    public OpenCvInternalCamera2.CameraDirection getCameraDirection() {
        return cameraDirection;
    }

    /**
     * Applicable only for internal camera
     *
     * @param cameraDirection
     */
    public void setCameraDirection(OpenCvInternalCamera2.CameraDirection cameraDirection) {
        this.cameraDirection = cameraDirection;
    }

    public OpenCvCamera.ViewportRenderingPolicy getLivePreviewRenderingPolicy() {
        return livePreviewRenderingPolicy;
    }

    /**
     * Applicable only for internal camera.
     * If you are using a an internal phone camera in a streaming orientation which does not match the current activity orientation,
     * you will find that the live preview is rendered 90 degrees out from how it "should be".
     * This does NOT impact how frames are delivered to your code.
     *
     * @param livePreviewRenderingPolicy
     */
    public void setLivePreviewRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy livePreviewRenderingPolicy) {
        this.livePreviewRenderingPolicy = livePreviewRenderingPolicy;
    }

    public OpenCvCamera.ViewportRenderer getViewportRenderer() {
        return viewportRenderer;
    }

    public void setViewportRenderer(OpenCvCamera.ViewportRenderer viewportRenderer) {
        this.viewportRenderer = viewportRenderer;
    }
}
