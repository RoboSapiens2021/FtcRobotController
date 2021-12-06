package org.firstinspires.ftc.teamcode.camera.opencv;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

/*
 * Draw a simple box around the middle 1/2 of the entire frame
 */
public class BoxAroundPipeline extends OpenCvPipeline {

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.rectangle(input, new Point(input.cols() / 4, input.rows() / 4), new Point(input.cols() * (3f / 4f), input.rows() * (3f / 4f)), new Scalar(0, 255, 0), 4);
        return input;
    }


}
