package org.firstinspires.ftc.teamcode.programs.subsystems;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class Camera extends OpenCvPipeline {

    private static final double MIN_AREA = 100.0;
    private static final int ENLARGEMENT_PIXELS = 15;

    // PUBLIC fields for OpMode use
    public double currentX = 0;   // horizontal offset from camera center
    public double currentY = 0;   // vertical offset from camera center
    public double targetX = 0;    // 0 = align to center
    public double targetY = 0;    // 0 = align to center

    private int frameCounter = 0;

    private static class BallData {
        int cx;
        int cy;
        double area;

        BallData(int cx, int cy, double area) {
            this.cx = cx;
            this.cy = cy;
            this.area = area;
        }
    }

    @Override
    public Mat processFrame(Mat input) {
        incrementFrameCounter();

        int imgW = input.cols();
        int imgH = input.rows();

        Mat imgHsv = new Mat();
        Imgproc.cvtColor(input, imgHsv, Imgproc.COLOR_BGR2HSV);

        // Cyan/green range (adjust if needed)
        Scalar cyanLow = new Scalar(20, 50, 50);
        Scalar cyanHigh = new Scalar(105, 255, 255);
        Scalar purpleLow = new Scalar (120, 50, 50);
        Scalar purpleHigh= new Scalar (170, 255, 255);
        Mat maskCyan = new Mat();
        Mat maskPurple = new Mat();
        ///  Cyan HSV
        Core.inRange(imgHsv, cyanLow, cyanHigh, maskCyan);

        imgHsv.release();

        List<MatOfPoint> contoursCyan = new ArrayList<>();
        Imgproc.findContours(maskCyan, contoursCyan, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        maskCyan.release();

        List<BallData> ballData = new ArrayList<>();
        processContours(input, contoursCyan, ballData, new Scalar(0, 255, 255), imgW, imgH);
        /// Purple HSV
        Core.inRange (imgHsv, purpleLow, purpleHigh, maskPurple);
        imgHsv.release ();
        List<MatOfPoint> contoursPurple = new ArrayList<>();
        Imgproc.findContours(maskPurple, contoursPurple, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        maskPurple.release ();
        processContours (input, contoursPurple, ballData, new Scalar (0, 255, 255), imgW, imgH);

        // Sliding window for max area (like before)
        double maxAreaSum = -1.0;
        int countInBestWindow = 0;
        Rect bestWindow = new Rect(0, 0, 200, 200);

        int WINDOW_WIDTH = 200;
        int WINDOW_HEIGHT = 200;
        int STEP_SIZE = 25;

        for (int xStart = 0; xStart <= imgW - WINDOW_WIDTH; xStart += STEP_SIZE) {
            for (int yStart = 0; yStart <= imgH - WINDOW_HEIGHT; yStart += STEP_SIZE) {

                double currentAreaSum = 0;
                int currentCount = 0;

                int xEnd = xStart + WINDOW_WIDTH;
                int yEnd = yStart + WINDOW_HEIGHT;

                for (BallData b : ballData) {
                    if (b.cx >= xStart && b.cx < xEnd && b.cy >= yStart && b.cy < yEnd) {
                        currentAreaSum += b.area;
                        currentCount++;
                    }
                }

                if (currentAreaSum > maxAreaSum) {
                    maxAreaSum = currentAreaSum;
                    countInBestWindow = currentCount;
                    bestWindow = new Rect(xStart, yStart, WINDOW_WIDTH, WINDOW_HEIGHT);
                }
            }
        }

        // Compute COM of best window
        if (countInBestWindow > 0) {
            int sumX = 0, sumY = 0;
            for (BallData b : ballData) {
                if (b.cx >= bestWindow.x && b.cx < bestWindow.x + bestWindow.width &&
                        b.cy >= bestWindow.y && b.cy < bestWindow.y + bestWindow.height) {
                    sumX += b.cx;
                    sumY += b.cy;
                }
            }

            int comX = sumX / countInBestWindow;
            int comY = sumY / countInBestWindow;

            currentX = comX - imgW / 2.0;  // offset from center
            currentY = comY - imgH / 2.0;

            // Draw for visualization
            Imgproc.rectangle(input, bestWindow, new Scalar(0, 255, 255), 2);
            Imgproc.circle(input, new Point(comX, comY), 5, new Scalar(255, 0, 0), -1);
        } else {
            currentX = 0;
            currentY = 0;
        }

        return input;
    }

    private void processContours(Mat image, List<MatOfPoint> contours,
                                 List<BallData> data, Scalar boxColor, int imgW, int imgH) {

        for (MatOfPoint c : contours) {
            double area = Imgproc.contourArea(c);

            if (area > MIN_AREA) {
                Rect b = Imgproc.boundingRect(c);
                int cx = b.x + b.width / 2;
                int cy = b.y + b.height / 2;
                data.add(new BallData(cx, cy, area));

                int x1 = Math.max(0, b.x - ENLARGEMENT_PIXELS);
                int y1 = Math.max(0, b.y - ENLARGEMENT_PIXELS);
                int x2 = Math.min(imgW - 1, b.x + b.width + ENLARGEMENT_PIXELS);
                int y2 = Math.min(imgH - 1, b.y + b.height + ENLARGEMENT_PIXELS);

                Imgproc.rectangle(image, new Point(x1, y1), new Point(x2, y2), boxColor, 2);
                Imgproc.circle(image, new Point(cx, cy), 3, new Scalar(0, 0, 255), -1);
            }

            c.release();
        }
    }

    private void incrementFrameCounter() {
        frameCounter++;
        if (frameCounter == 100) System.out.println("100 frames processed");
        if (frameCounter >= 200) frameCounter = 0;
    }
}
