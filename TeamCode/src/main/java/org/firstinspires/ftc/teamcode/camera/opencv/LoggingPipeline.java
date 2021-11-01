package org.firstinspires.ftc.teamcode.camera.opencv;

import org.firstinspires.ftc.teamcode.util.Logger;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class LoggingPipeline extends OpenCvPipeline {
    private Logger logger = Logger.getInstance();

    @Override
    public void init(Mat mat) {
        super.init(mat);
        logger.info("init called");
    }

    @Override
    public Mat processFrame(Mat input) {
        logger.info("processFrame called");
        return input;
    }

    public void onViewportTapped() {
        logger.info("ViewportTapped");
        super.onViewportTapped();
    }
}
