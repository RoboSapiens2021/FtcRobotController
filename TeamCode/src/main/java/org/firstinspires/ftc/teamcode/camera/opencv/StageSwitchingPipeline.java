package org.firstinspires.ftc.teamcode.camera.opencv;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/*
 * With this pipeline, we demonstrate how to change which stage of is rendered to the viewport when the viewport is tapped.
 * This is particularly useful during pipeline development.
 */
public class StageSwitchingPipeline extends OpenCvPipeline {
    private Mat yCbCrChan2Mat = new Mat();
    private Mat thresholdMat = new Mat();
    private Mat contoursOnFrameMat = new Mat();
    private List<MatOfPoint> contoursList = new ArrayList<>();
    private int numContoursFound;

    enum Stage {
        YCbCr_CHAN2,
        THRESHOLD,
        CONTOURS_OVERLAYED_ON_FRAME,
        RAW_IMAGE,
    }

    private Stage stageToRenderToViewport = Stage.YCbCr_CHAN2;
    private Stage[] stages = Stage.values();


    /**
     * Note that this method is invoked from the UI thread
     * so whatever we do here, we must do quickly.
     */
    public void onViewportTapped() {
        int currentStageNum = stageToRenderToViewport.ordinal();
        int nextStageNum = currentStageNum + 1;
        if (nextStageNum >= stages.length) {
            nextStageNum = 0;
        }
        stageToRenderToViewport = stages[nextStageNum];
    }

    @Override
    /*
     * This pipeline finds the contours of yellow blobs such as the Gold Mineral from the Rover Ruckus game.
     */
    public Mat processFrame(Mat input) {
        contoursList.clear();

        Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);
        Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);
        Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        numContoursFound = contoursList.size();
        input.copyTo(contoursOnFrameMat);
        Imgproc.drawContours(contoursOnFrameMat, contoursList, -1, new Scalar(0, 0, 255), 3, 8);

        switch (stageToRenderToViewport) {
            case YCbCr_CHAN2: {
                return yCbCrChan2Mat;
            }

            case THRESHOLD: {
                return thresholdMat;
            }

            case CONTOURS_OVERLAYED_ON_FRAME: {
                return contoursOnFrameMat;
            }

            case RAW_IMAGE: {
                return input;
            }

            default: {
                return input;
            }
        }
    }

    public int getNumContoursFound() {
        return numContoursFound;
    }
}
