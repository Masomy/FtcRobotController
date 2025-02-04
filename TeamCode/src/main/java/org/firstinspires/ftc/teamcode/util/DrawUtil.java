package org.firstinspires.ftc.teamcode.util;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class DrawUtil {

    public static void drawText(Mat output, String text, Point point, Scalar color) {
        Imgproc.putText(
                output,
                text,
                point,
                Imgproc.FONT_HERSHEY_COMPLEX, 1.0,
                color,
                3
        );
    }

    public static void drawLine(Mat output, Point point1, Point point2, Scalar color) {
        Imgproc.line(output, point1, point2, color, 2);
    }

}
