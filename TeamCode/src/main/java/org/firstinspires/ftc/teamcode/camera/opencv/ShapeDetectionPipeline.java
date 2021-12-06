package org.firstinspires.ftc.teamcode.camera.opencv;

import android.os.Build;

import androidx.annotation.RequiresApi;

import org.firstinspires.ftc.teamcode.util.Logger;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class ShapeDetectionPipeline extends OpenCvPipeline {
    private static Logger logger = Logger.getInstance();

    @Override
    @RequiresApi(api = Build.VERSION_CODES.N)
    public Mat processFrame(Mat originalImage) {
        final Mat processedImage = new Mat(originalImage.height(), originalImage.width(), originalImage.type());
        // Blur an image using a Gaussian filter
        Imgproc.GaussianBlur(originalImage, processedImage, new Size(7, 7), 1);

        // Switch from RGB to GRAY
        Imgproc.cvtColor(processedImage, processedImage, Imgproc.COLOR_RGB2GRAY);

        // Find edges in an image using the Canny algorithm
        Imgproc.Canny(processedImage, processedImage, 200, 25);

        // Dilate an image by using a specific structuring element
        // https://en.wikipedia.org/wiki/Dilation_(morphology)
        Imgproc.dilate(processedImage, processedImage, new Mat(), new Point(-1, -1), 1);

        markOuterContour(processedImage, originalImage);
        return processedImage;
    }

    /**
     * Used to mark outer rectangle and its corners.
     *
     * @param processedImage Image used for calculation of contours and corners.
     * @param originalImage  Image on which marking is done.
     */
    @RequiresApi(api = Build.VERSION_CODES.N)
    private void markOuterContour(final Mat processedImage, final Mat originalImage) {

        // Find contours of an image
        final List<MatOfPoint> allContours = new ArrayList<>();
        Imgproc.findContours(processedImage, allContours, new Mat(processedImage.size(), processedImage.type()), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
        logger.info("allContours size: " + allContours.size());
        // Filter out noise and display contour area value
        final List<MatOfPoint> filteredContours = allContours.stream()
                .filter(contour -> {
                    final double contourArea = Imgproc.contourArea(contour);

                    final Rect rect = Imgproc.boundingRect(contour);

                    final boolean isNotNoise = contourArea > 14000;
                    if (isNotNoise) {
                        System.out.println("contourArea: "+contourArea);
                        Imgproc.putText(originalImage, "Area: " + (int) contourArea, new Point(rect.x + rect.width, rect.y + rect.height), 2, 0.5, new Scalar(124, 252, 0), 1);

                        MatOfPoint2f dst = new MatOfPoint2f();
                        contour.convertTo(dst, CvType.CV_32F);
                        Imgproc.approxPolyDP(dst, dst, 0.02 * Imgproc.arcLength(dst, true), true);
                        Imgproc.putText(originalImage, "Points: " + dst.toArray().length, new Point(rect.x + rect.width, rect.y + rect.height + 15), 2, 0.5, new Scalar(124, 252, 0), 1);
                    }

                    return isNotNoise;
                }).collect(Collectors.toList());

        // Mark contours
        // Negative value indicates that we want to draw all of contours
        Imgproc.drawContours(originalImage, filteredContours, -1, new Scalar(124, 252, 0), // Green color
                1
        );
    }



}
