package org.firstinspires.ftc.teamcode.robot.components.vision.detector;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Typeface;
import android.text.TextPaint;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.opencv.core.Mat;
import org.opencv.core.Rect;

import java.util.Locale;
import java.util.Map;


public class ObjectDetectionVisionProcessor implements org.firstinspires.ftc.vision.VisionProcessor {
    ObjectDetector objectDetector = new ObjectDetector(0, RobotConfig.X_PIXEL_COUNT, 200, RobotConfig.Y_PIXEL_COUNT);
    private final TextPaint textPaint = new TextPaint();
    private final Paint greenLinePaint = new Paint();
    private final Paint redLinePaint = new Paint();
    private final Paint blueLinePaint = new Paint();


    public ObjectDetectionVisionProcessor() {
        // setting up the paint for the text in the center of the box
        textPaint.setColor(Color.GREEN); // you may want to change this
        textPaint.setTextAlign(Paint.Align.LEFT);
        textPaint.setAntiAlias(true);
        textPaint.setTextSize(15); // or this
        textPaint.setTypeface(Typeface.DEFAULT_BOLD);

        // setting up the paint for the lines that comprise the box
        greenLinePaint.setColor(Color.GREEN); // you may want to change this
        greenLinePaint.setAntiAlias(true);
        greenLinePaint.setStrokeWidth(2); // or this
        greenLinePaint.setStrokeCap(Paint.Cap.ROUND);
        greenLinePaint.setStrokeJoin(Paint.Join.ROUND);

        // setting up the paint for the lines that comprise the area of interes
        redLinePaint.setColor(Color.RED); // you may want to change this
        redLinePaint.setAntiAlias(true);
        redLinePaint.setStrokeWidth(2); // or this
        redLinePaint.setStrokeCap(Paint.Cap.ROUND);
        redLinePaint.setStrokeJoin(Paint.Join.ROUND);

        // setting up the paint for the lines that comprise the object at cross hairs
        blueLinePaint.setColor(Color.BLUE); // you may want to change this
        blueLinePaint.setAntiAlias(true);
        blueLinePaint.setStrokeWidth(2); // or this
        blueLinePaint.setStrokeCap(Paint.Cap.ROUND);
        blueLinePaint.setStrokeJoin(Paint.Join.ROUND);
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        return objectDetector.process(frame);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // this method draws the rectangle around the largest contour and puts the current position into that rectangle
        // you don't need to call it

        //draw rectangle around area of interest
        Rect areaOfInterest = objectDetector.getAreaOfInterest();
        drawRectangle(canvas, scaleBmpPxToCanvasPx, redLinePaint, areaOfInterest);
        Map<ObjectDetector.ObjectType, DetectableObject> detectedObjects = (Map<ObjectDetector.ObjectType, DetectableObject>) userContext;
        //paint information about each of the largest objects seen
        synchronized (detectedObjects) {
            for (DetectableObject detectableObject : detectedObjects.values()) {
                if (detectableObject.getType() == ObjectDetector.ObjectType.CrossHair) {
                    paintObject(canvas, scaleBmpPxToCanvasPx, blueLinePaint, textPaint, detectableObject);
                } else {
                    paintObject(canvas, scaleBmpPxToCanvasPx, greenLinePaint, textPaint, detectableObject);
                }
            }
        }
    }

    private static void paintObject(Canvas canvas, float scaleBmpPxToCanvasPx, Paint linePaint, TextPaint textPaint, DetectableObject detectableObject) {
        Rect rect = detectableObject.getBoundingRectangleOfLargestObject();
        if (rect != null) {
            drawRectangle(canvas, scaleBmpPxToCanvasPx, linePaint, rect);
            String text = String.format(Locale.getDefault(), "%s @%d,%d",
                    detectableObject.getShortName(),
                    rect.x,
                    rect.y);
            textPaint.setColor(linePaint.getColor());
            canvas.drawText(text,
                    (float) rect.x * scaleBmpPxToCanvasPx,
                    (float) rect.y * scaleBmpPxToCanvasPx,
                    textPaint);
        }
    }

    public static void drawRectangle(Canvas canvas, float scaleBmpPxToCanvasPx, Paint linePaint, Rect rect) {
        float[] points = {rect.x * scaleBmpPxToCanvasPx, rect.y * scaleBmpPxToCanvasPx, (rect.x + rect.width) * scaleBmpPxToCanvasPx, (rect.y + rect.height) * scaleBmpPxToCanvasPx};

        canvas.drawLine(points[0], points[1], points[0], points[3], linePaint);
        canvas.drawLine(points[0], points[1], points[2], points[1], linePaint);

        canvas.drawLine(points[0], points[3], points[2], points[3], linePaint);
        canvas.drawLine(points[2], points[1], points[2], points[3], linePaint);
    }

    public void decrementMinX() {
        objectDetector.decrementMinAllowedX();
    }

    public void incrementMinX() {
        objectDetector.incrementMinAllowedX();
    }

    public void decrementMaxX() {
        objectDetector.decrementMaxAllowedX();
    }

    public void incrementMaxX() {
        objectDetector.incrementMaxAllowedX();
    }

    public void decrementMinY() {
        objectDetector.decrementMinAllowedY();
    }

    public void incrementMinY() {
        objectDetector.incrementMinAllowedY();
    }

    public void decrementMaxY() {
        objectDetector.decrementMaxAllowedY();
    }

    public void incrementMaxY() {
        objectDetector.incrementMaxAllowedY();
    }

    public double getXPositionOfLargestObject(ObjectDetector.ObjectType objectType) {
        return objectDetector.getXPositionOfLargestObject(objectType);
    }

    public double getYPositionOfLargestObject(ObjectDetector.ObjectType objectType) {
        return objectDetector.getYPositionOfLargestObject(objectType);
    }

    public double getWidthOfLargestObject(ObjectDetector.ObjectType objectType) {
        return objectDetector.getWidthOfLargestObject(objectType);
    }

    public double getHeightOfLargestObject(ObjectDetector.ObjectType objectType) {
        return objectDetector.getHeightOfLargestObject(objectType);
    }

    public boolean seeingObject(ObjectDetector.ObjectType objectName) {
        return getLargestArea(objectName) > 0;
    }

    public double getLargestArea(ObjectDetector.ObjectType objectName) {
        return objectDetector.getLargestArea(objectName);
    }

    public double getDistanceToLargestObject(ObjectDetector.ObjectType objectType) {
        return objectDetector.getDistanceFromCameraOfLargestObject(objectType);
    }

    public void manageVisibility(Gamepad gamepad1, Gamepad gamepad2) {
        this.objectDetector.manageObjectDetection(gamepad1, gamepad2);
    }

    public String getStatus() {
        return objectDetector.getStatus();
    }

    public void enableObject(ObjectDetector.ObjectType type) {
        this.objectDetector.enableObject(type);
    }

    public void disableObject(ObjectDetector.ObjectType type) {
        this.objectDetector.disableObject(type);
    }

}
