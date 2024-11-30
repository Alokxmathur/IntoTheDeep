package org.firstinspires.ftc.teamcode.robot.components.vision;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.components.vision.detector.ObjectDetectionVisionProcessor;
import org.firstinspires.ftc.teamcode.robot.components.vision.detector.ObjectDetector;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.util.List;
import java.util.Locale;

public class SilverTitansVisionPortal {
    org.firstinspires.ftc.vision.VisionPortal visionPortal;
    AprilTagProcessor aprilTagProcessor;
    ColorBlobLocatorProcessor blueColorLocator = new ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(ColorRange.BLUE)         // use a predefined color match
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
            .setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5))  // search central 1/4 of camera view
            .setDrawContours(true)                        // Show contours on the Stream Preview
            .setBlurSize(5)                               // Smooth the transitions between different colors in image
            .setBoxFitColor(Color.RED)
            .build();
    ColorBlobLocatorProcessor redColorLocator = new ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(ColorRange.RED)         // use a predefined color match
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
            .setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5))  // search central 1/4 of camera view
            .setDrawContours(true)                        // Show contours on the Stream Preview
            .setBlurSize(5)                               // Smooth the transitions between different colors in image
            .setBoxFitColor(Color.BLUE)
            .build();

    ColorBlobLocatorProcessor yellowColorLocator = new ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(ColorRange.YELLOW)         // use a predefined color match
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
            .setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5))  // search central 1/4 of camera view
            .setDrawContours(true)                        // Show contours on the Stream Preview
            .setBlurSize(5)                               // Smooth the transitions between different colors in image
            .setBoxFitColor(Color.WHITE)
            .build();

    public void init(HardwareMap hardwareMap) {
        this.aprilTagProcessor = new AprilTagProcessor.Builder().build();
        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTagProcessor.setDecimation(2);

        visionPortal = new org.firstinspires.ftc.vision.VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, RobotConfig.WEBCAM_ID))
                .setCameraResolution(new android.util.Size(RobotConfig.X_PIXEL_COUNT, RobotConfig.Y_PIXEL_COUNT))
                .addProcessors(aprilTagProcessor, blueColorLocator, redColorLocator, yellowColorLocator)
                .build();
    }
    public String getStatus() {
        ColorBlobLocatorProcessor.Blob redBlob = getLargestBlob(redColorLocator);
        ColorBlobLocatorProcessor.Blob blueBlob = getLargestBlob(blueColorLocator);
        ColorBlobLocatorProcessor.Blob yellowBlob = getLargestBlob(yellowColorLocator);
        return String.format(Locale.getDefault(),
                "Red:%s,Blue:%s,Yellow:%s",
                getBlobStatus(redBlob), getBlobStatus(blueBlob), getBlobStatus(yellowBlob));
    }

    public ColorBlobLocatorProcessor.Blob getYellowObject() {
        return getLargestBlob(this.yellowColorLocator);
    }

    public ColorBlobLocatorProcessor.Blob getRedObject() {
        return getLargestBlob(this.redColorLocator);
    }
    public ColorBlobLocatorProcessor.Blob getBlueObject() {
        return getLargestBlob(this.blueColorLocator);
    }
    /**
     * Returns a string representation of a blob
     * @param blob
     * @return
     */
    public String getBlobStatus(ColorBlobLocatorProcessor.Blob blob) {
        if (blob != null) {
            RotatedRect box = blob.getBoxFit();
            return String.format(Locale.getDefault(), "@%d,%d of area:%d",
                    box.center.x, box.center.y, box.size.area());
        }
        else {
            return "not seeing";
        }
    }
    private ColorBlobLocatorProcessor.Blob getLargestBlob(ColorBlobLocatorProcessor processor) {
        // Read the current list
        List<ColorBlobLocatorProcessor.Blob> blobs = processor.getBlobs();

        /*
         * The list of Blobs can be filtered to remove unwanted Blobs.
         *   Note:  All contours will be still displayed on the Stream Preview, but only those that satisfy the filter
         *          conditions will remain in the current list of "blobs".  Multiple filters may be used.
         *
         * Use any of the following filters.
         *
         * ColorBlobLocatorProcessor.Util.filterByArea(minArea, maxArea, blobs);
         *   A Blob's area is the number of pixels contained within the Contour.  Filter out any that are too big or small.
         *   Start with a large range and then refine the range based on the likely size of the desired object in the viewfinder.
         *
         * ColorBlobLocatorProcessor.Util.filterByDensity(minDensity, maxDensity, blobs);
         *   A blob's density is an indication of how "full" the contour is.
         *   If you put a rubber band around the contour you would get the "Convex Hull" of the contour.
         *   The density is the ratio of Contour-area to Convex Hull-area.
         *
         * ColorBlobLocatorProcessor.Util.filterByAspectRatio(minAspect, maxAspect, blobs);
         *   A blob's Aspect ratio is the ratio of boxFit long side to short side.
         *   A perfect Square has an aspect ratio of 1.  All others are > 1
         */
        ColorBlobLocatorProcessor.Util.filterByArea(100, 20000, blobs);  // filter out very small blobs.
        ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);
        return blobs.get(0);
    }
    /**
     * Add telemetry about AprilTag detections.
     */
    public void telemetryAprilTag(Telemetry telemetry) {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

    }   // end method telemetryAprilTag()

    public List<AprilTagDetection>  getAprilTags() {
        return this.aprilTagProcessor.getDetections();
    }
}
