package org.firstinspires.ftc.teamcode.glowCode;

import android.media.ImageWriter;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.utils.Converters;
import org.opencv.core.Size;
import org.opencv.core.Core;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


//this will be used later
public class OpenCVStuff {
    private final HardwareMapping robot = new HardwareMapping();

    public static Mat makeEdged(Mat inputMat) {
        Mat grayscaleImage = new Mat();
        Imgproc.cvtColor(inputMat, grayscaleImage, Imgproc.COLOR_BGR2GRAY);
        Mat blurredImage = new Mat();
        Imgproc.GaussianBlur(grayscaleImage, blurredImage, new Size(5, 5), 0);
        Mat edgedImage = new Mat();
        Imgproc.Canny(blurredImage, edgedImage, 50, 200, 255);
        return edgedImage;
    }

    public static MatOfPoint findLargestContour(Mat image) {
        List<MatOfPoint> contour = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(image, contour, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        double largestContourSize = 0;
        int largestContour = 0;
        for (int i = 0; i < contour.size(); i++) {
            if (Imgproc.contourArea(contour.get(i)) > largestContourSize) {
                largestContour = i;
                largestContourSize = Imgproc.contourArea(contour.get(i));
            }
        }
        return contour.get(largestContour);
    }

    public static int detectNumber(Mat inputtedImage) {
        Mat image1 = Imgcodecs.imread("![](../../../../../../../../../FtcRobotController/src/main/assets/IMG_2811.jpg)");
        Mat image2 = Imgcodecs.imread("![](../../../../../../../../../FtcRobotController/src/main/assets/IMG_2810.jpg)");
        Mat image3 = Imgcodecs.imread("![](../../../../../../../../../FtcRobotController/src/main/assets/IMG_2812.jpg)");
        Mat comparableImage1 = makeEdged(image1);
        Mat comparableImage2 = makeEdged(image2);
        Mat comparableImage3 = makeEdged(image3);
        /*MatOfPoint contour1 = findLargestContour(comparableImage1);
        MatOfPoint contour2 = findLargestContour(comparableImage2);
        MatOfPoint contour3 = findLargestContour(comparableImage3);*/
        Mat edgedImage = makeEdged(inputtedImage);
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edgedImage, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        final int magicNumber = 100;
        for (int i = 0; i < contours.size(); i++) {
            if (Imgproc.contourArea(contours.get(i)) > magicNumber) {
                if (Imgproc.matchShapes(edgedImage, comparableImage1, 1, 1) < 0.5) {
                    return 1;
                } else if (Imgproc.matchShapes(edgedImage, comparableImage2, 1, 1) < 0.5) {
                    return 2;
                } else if (Imgproc.matchShapes(edgedImage, comparableImage3, 1, 1) < 0.5) {
                    return 3;
                }
                return 4;
            }
        }
        return 0;
    }
}
