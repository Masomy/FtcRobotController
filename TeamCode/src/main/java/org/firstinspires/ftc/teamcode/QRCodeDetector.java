package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.util.PointUtil;
import org.firstinspires.ftc.teamcode.util.RectUtil;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;

import java.util.ArrayList;
import java.util.List;

public class QRCodeDetector {
    private static final int THRESHOLD = 3;

    private static final double EXPECTED_SIZE_IN_PIXELS = 60;
    private static final double SIZE_THRESHOLD = 40;

    private static final double EXPECTED_ANGLE = 90.0;
    private static final double ANGLE_THRESHOLD = 10.0;

    private int detectionCount;
    private int detectionAttempts;
    private Rect detectionBounds;
    private Point center;
    private List<List<Point>> detectedPoints;
    private List<String> detectedData;

    private org.opencv.objdetect.QRCodeDetector detector = new org.opencv.objdetect.QRCodeDetector();

    //Initializes a QR code detector.
    public QRCodeDetector(Point center, int size) {
        this.center = center;
        //creates a rectangle of the given size with its center at the given point
        this.detectionBounds = new Rect(
                (int) (center.x - size / 2),
                (int) center.y - size / 2,
                size, size
        );
        reset();
    }

    public void reset() {
        //resets the QR code detector so it can be used again.
        detectionCount = 0;
        detectionAttempts = 0;
        detectedPoints = new ArrayList<>();
        detectedData = new ArrayList<>();
    }

    public boolean isDetected() {
        //if we have detected more than or equal to the number of detections we want
        //return true
        return detectionCount >= THRESHOLD;
    }

    public int getDetectionCount() {
        return detectionCount;
    }

    public Rect getDetectionBounds() {
        return detectionBounds;
    }

    public Point getCenter() {
        return center;
    }

    public List<List<Point>> getDetectedPoints() {
        return detectedPoints;
    }

    public List<String> getDetectedData() {
        return detectedData;
    }

    public double getDetectionPercentage() {
        return ((double) detectionCount) / detectionAttempts;
    }

    public void processFrame(Mat input) {
        //if we detect a QR code then add 1 to the detection count
        //regardless add one to the detection attempts
        if (detect(input)) {
            detectionCount++;
        }
        detectionAttempts++;
    }

    //image is passed in from the webcam
    private boolean detect(Mat image) {
        //makes a rectangle that determines where the image is cropped
        Rect bounds = detectionBounds.clone();
        RectUtil.clip(bounds, image.size());

        //makes a new cropped image out of the image
        Mat cropped;
        cropped = new Mat(image, bounds);

        //calls the opencv to detect a QR code on the image and returns the points
        //of the QR code
        Mat result = new Mat();
        detector.detect(cropped, result);

        //if we have points meaning we found the QR code then return true,
        //otherwise return false

        if (result.rows() > 0) {
            List<Point> detection = new ArrayList<>();

            for (int i = 0; i < result.cols(); i++) {
                Point point = new Point(result.get(0, i));
                point.x += detectionBounds.x;
                point.y += detectionBounds.y;
                detection.add(point);
            }

            if (validDetection(detection)) {
                detectedPoints.add(detection);
                return true;
            }
        }
        return false;
    }

    private boolean validDetection(List<Point> points) {
        for (int i = 0; i < points.size(); i++) {
            Point pt1 = points.get(i);
            Point pt2 = points.get((i + 1) % points.size());
            //Point pt3 = points.get((i + 2) % points.size());
            double length = PointUtil.distance(pt1, pt2);
            //double angle = calculateAngle(pt1, pt2, pt3);
            if (Math.abs(EXPECTED_SIZE_IN_PIXELS - length) > SIZE_THRESHOLD) {
                return false;
            }

            /*if (Math.abs(EXPECTED_ANGLE - angle) > ANGLE_THRESHOLD) {
                return false;
            }*/
        }
        return true;
    }


    /*private double calculateAngle(Point p1, Point p2, Point p3) {

        double numerator = p2.y * (p1.x - p3.x) + p1.y * (p3.x - p2.x) + p3.y * (p2.x - p1.x);
        double denominator = (p2.x - p1.x) * (p1.x - p3.x) + (p2.y - p1.y) * (p1.y - p3.y);
        double ratio = numerator / denominator;

        double angleRad = Math.atan(ratio);
        double angleDeg = (angleRad * 180) / Math.PI;

        if (angleDeg < 0) {
            angleDeg = 180 + angleDeg;
        }

        return angleDeg;
    }*/
}
