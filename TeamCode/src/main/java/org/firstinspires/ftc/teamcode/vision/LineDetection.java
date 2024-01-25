package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.helpers.Prop;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class LineDetection implements VisionProcessor, CameraStreamSource {

    Telemetry telemetry;

    private List<Prop> detectedProps = new ArrayList<>();

    //Streaming
    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1,1,Bitmap.Config.RGB_565));

    public LineDetection(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Executed before the first call to processFrame

        //Streaming
        lastFrame.set(Bitmap.createBitmap(width,height,Bitmap.Config.RGB_565));
    }

    @Override
    public Mat processFrame(Mat input, long captureTimeNanos) {
        // Executed every time a new frame is dispatched

        //HSV Layer
        Mat hsv = new Mat();
        Imgproc.cvtColor(input,hsv,Imgproc.COLOR_RGB2HSV);

        //Color Threshold
        Mat blueThresh = new Mat();
        Scalar blueLow = new Scalar(100,70,50);
        Scalar blueHigh = new Scalar(150,255,255);
        Core.inRange(hsv,blueLow,blueHigh,blueThresh);

        Mat redThresh = new Mat();

        Mat redThreshPart1 = new Mat();
        Scalar redLow = new Scalar(170,70,50);
        Scalar redHigh = new Scalar(180,255,255);
        Core.inRange(hsv,redLow,redHigh,redThreshPart1);

        Mat redThreshPart2 = new Mat();
        Scalar red2Low = new Scalar(0,70,50);
        Scalar red2High = new Scalar(10,255,255);
        Core.inRange(hsv,red2Low,red2High,redThreshPart2);

        Core.bitwise_or(redThreshPart1,redThreshPart2,redThresh);

        //Adding everything to the combined threshold

        //Releasing all original thresholds
        redThreshPart1.release();
        redThreshPart2.release();

        //Finding contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(redThresh, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
        Imgproc.findContours(blueThresh, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);

        //Clear detected props list
        detectedProps.clear();

        //Looping though contours
        for(int i = 0; i < contours.size(); i ++) {
            //Copying contour
            MatOfPoint2f copy = new MatOfPoint2f(contours.get(i).toArray());
            Rect rect = Imgproc.boundingRect(copy);
            double area = Imgproc.contourArea(copy);

            //THe prep
            if (area>2600) {
                Imgproc.rectangle(input,rect,new Scalar(145,14,220),2);
                detectedProps.add(new Prop(rect.x,rect.y,area));
            }


            //Releasing the contour
            copy.release();
        }


        //Releasing
        hsv.release();
        blueThresh.release();
        redThresh.release();


        //Streaming
        Bitmap b = Bitmap.createBitmap(input.width(),input.height(),Bitmap.Config.RGB_565);
        Utils.matToBitmap(input,b);
        lastFrame.set(b);

        //Output frame
        return input; // Return the image that will be displayed in the viewport
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext) {

    }

    //Streaming
    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }

    //Return Valyes
    public List<Prop> getDetectedProps() {
        return detectedProps;
    }
}