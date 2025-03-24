package org.firstinspires.ftc.teamcode.robot.components.vision;

import android.graphics.Color;

import com.qualcomm.hardware.sparkfun.SparkFunLEDStick;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;

import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

public class SilverTitansVisionPortal {
    public enum SamplesSeen {
        YellowOnly, AllianceAndYellow, YellowAndAlliance, Nothing
    }
    VisionPortal visionPortalAprilTags, visionPortalSamples;
    AprilTagProcessor aprilTagProcessor;

    private SparkFunLEDStick ledStickLeft, ledStickRight;
    private SamplesSeen lastSamplesSeen = SamplesSeen.Nothing;

    public static final int intensity = 6;
    ColorBlobLocatorProcessor blueColorLocator = new ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(ColorRange.BLUE)         // use a predefined color match
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
            .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))  // search central 1/4 of camera view
            .setDrawContours(false)                        // do not show contours on the Stream Preview
            .setBlurSize(5)                               // Smooth the transitions between different colors in image
            .setBoxFitColor(Color.RED)
            .build();
    ColorBlobLocatorProcessor redColorLocator = new ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(ColorRange.RED)         // use a predefined color match
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
            .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))  // search central 1/4 of camera view
            .setDrawContours(false)                        // do not show contours on the Stream Preview
            .setBlurSize(5)                               // Smooth the transitions between different colors in image
            .setBoxFitColor(Color.BLUE)
            .build();

    ColorBlobLocatorProcessor yellowColorLocator = new ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(new ColorRange(ColorSpace.HSV, new Scalar(10, 120, 40), new Scalar(30, 255, 255)))
            //.setTargetColorRange(ColorRange.YELLOW)         // use a predefined color match
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
            .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))  // search central 1/4 of camera view
            .setDrawContours(false)                        // do not show contours on the Stream Preview
            .setBlurSize(5)                               // Smooth the transitions between different colors in image
            .setBoxFitColor(Color.WHITE)
            .build();

    public void init(HardwareMap hardwareMap) {
        //initialize the led strips
        ledStickLeft = hardwareMap.get(SparkFunLEDStick.class, "leftLED");
        ledStickLeft.resetDeviceConfigurationForOpMode();
        ledStickLeft.setBrightness(0);

        ledStickRight = hardwareMap.get(SparkFunLEDStick.class, "rightLED");
        ledStickRight.resetDeviceConfigurationForOpMode();
        ledStickRight.setBrightness(0);

        // Because we want to show two camera feeds simultaneously, we need to inform
        // the SDK that we want it to split the camera monitor area into two smaller
        // areas for us. It will then give us View IDs which we can pass to the individual
        // vision portals to allow them to properly hook into the UI in tandem.
        int[] viewIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);

        this.aprilTagProcessor = new AprilTagProcessor.Builder().build();
        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTagProcessor.setDecimation(1);

        visionPortalAprilTags = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, RobotConfig.WEBCAM1))
                .addProcessors(aprilTagProcessor)
                //.setCameraResolution(new android.util.Size(1920, 1080))
                //.setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setLiveViewContainerId(viewIds[0])
                .build();
        visionPortalSamples = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, RobotConfig.WEBCAM2))
                .addProcessors(blueColorLocator, redColorLocator, yellowColorLocator)
                .setCameraResolution(new android.util.Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setLiveViewContainerId(viewIds[1])
                .build();
        /*
        visionPortalSamples.setProcessorEnabled(blueColorLocator, false);
        visionPortalSamples.setProcessorEnabled(redColorLocator, false);
        visionPortalSamples.setProcessorEnabled(yellowColorLocator, false);

         */
        setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
    }
    public String getStatus() {
        ColorBlobLocatorProcessor.Blob redBlob = getLargestBlob(redColorLocator);
        ColorBlobLocatorProcessor.Blob blueBlob = getLargestBlob(blueColorLocator);
        ColorBlobLocatorProcessor.Blob yellowBlob = getLargestBlob(yellowColorLocator);
        giveLightFeedback(redBlob, yellowBlob, blueBlob);
        return String.format(Locale.getDefault(),
                "AprilTags:%d, Red:%s,Blue:%s,Yellow:%s",
                getAprilTags().size(),
                getBlobStatus(redBlob),
                getBlobStatus(blueBlob),
                getBlobStatus(yellowBlob));
    }

    private void giveLightFeedback(ColorBlobLocatorProcessor.Blob redBlob, ColorBlobLocatorProcessor.Blob yellowBlob, ColorBlobLocatorProcessor.Blob blueBlob) {
        ColorBlobLocatorProcessor.Blob allianceBlobToUse = Match.getInstance().getAlliance() == Alliance.Color.RED ?
                redBlob : blueBlob;
        if (allianceBlobToUse != null) {
            if (yellowBlob != null) {
                //we are seeing alliance sample and yellow
                setLedColors(SamplesSeen.YellowAndAlliance);
            }
            else {
                //only seeing alliance sample - no yellow
                setLedColors(SamplesSeen.AllianceAndYellow);
            }
        }
        else {
            //not seeing alliance sample but seeing yellow
            if (yellowBlob != null) {
                //seeing yellow
                setLedColors(SamplesSeen.YellowOnly);
            }
            else {
                //seeing neither red not yellow
                setLedColors(SamplesSeen.Nothing);
            }
        }
    }

    private void setLedColor(int color) {
        if (color == Color.WHITE) {
            ledStickLeft.setBrightness(0);
            ledStickRight.setBrightness(0);
        }
        else {
            ledStickLeft.setBrightness(intensity);
            ledStickLeft.setColor(color);

            ledStickRight.setBrightness(intensity);
            ledStickRight.setColor(color);
        }
    }
    private void setLedColors(int[] colors) {
        ledStickLeft.setColors(colors);
        ledStickRight.setColors(colors);
    }

    private void setLedColors(SamplesSeen samplesSeen) {
        if (samplesSeen != lastSamplesSeen) {
            lastSamplesSeen = samplesSeen;
            switch (samplesSeen) {
                case YellowOnly: {
                    setLedColor(Color.YELLOW);
                    break;
                }
                case Nothing: {
                    setLedColor(Color.WHITE);
                    break;
                }
                case YellowAndAlliance: {
                    setLedColor(Color.rgb(255,0, 255));
                    break;
                }
                case AllianceAndYellow: {
                    setLedColor(Match.getInstance().getAlliance() == Alliance.Color.RED
                            ? Color.RED : Color.BLUE);
                    break;
                }
            }
        }
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
    public static double getSampleAngle(ColorBlobLocatorProcessor.Blob blob) {
        if (blob != null) {
            RotatedRect box = blob.getBoxFit();
            Point[] points = new Point[4];
            box.points(points);
            //add 90 degrees because of the way the angle is returned by opencv
            double angle;
            double p1ToP2Distance = Math.sqrt(Math.pow(points[0].x - points[1].x, 2) + Math.pow(points[0].y - points[1].y, 2));
            double p2ToP3Distance = Math.sqrt(Math.pow(points[1].x - points[2].x, 2) + Math.pow(points[1].y - points[2].y, 2));
            if (p1ToP2Distance > p2ToP3Distance) {
                angle = 90 - Math.toDegrees(Math.atan2(points[2].y-points[1].y, points[2].x-points[1].x));
            }
            else {
                angle = 90 - Math.toDegrees(Math.atan2(points[1].y-points[0].y, points[1].x-points[0].x));
            }
            return angle;
        }
        else {
            return -500;
        }
    }
    /**
     * Returns a string representation of a blob
     * @param blob - a contour seen by openCV
     * @return a string representation of the blob
     */
    public String getBlobStatus(ColorBlobLocatorProcessor.Blob blob) {
        if (blob != null) {
            double angle = getSampleAngle(blob);
            RotatedRect box = blob.getBoxFit();
            return String.format(Locale.getDefault(), "@:x%d-%d,y%d-%d at angle: %.2f",
                    (int)(box.center.x-(box.size.height/2)), (int)(box.center.x+(box.size.height/2)),
                    (int)(box.center.y-(box.size.width/2)), (int)(box.center.y+(box.size.width/2)),
                    angle);
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
        ColorBlobLocatorProcessor.Util.filterByArea(100000, 1280*720, blobs);  // filter out very small blobs.
        //ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);

        if (!blobs.isEmpty()) {
            return blobs.get(0);
        }
        else {
            return null;
        }
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
                telemetry.addLine(String.format(Locale.getDefault(),"\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format(Locale.getDefault(), "XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format(Locale.getDefault(), "PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format(Locale.getDefault(), "RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format(Locale.getDefault(),"\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format(Locale.getDefault(),"Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

    }   // end method telemetryAprilTag()

    public List<AprilTagDetection>  getAprilTags() {
        return this.aprilTagProcessor.getDetections();
    }
    /*
 Manually set the camera gain and exposure.
 This can only be called AFTER calling initAprilTag(), and only works for Webcams;
*/
    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortalAprilTags == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortalAprilTags.getCameraState() != VisionPortal.CameraState.STREAMING) {
            while (visionPortalAprilTags.getCameraState() != VisionPortal.CameraState.STREAMING) {
                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
        }

            ExposureControl exposureControl = visionPortalAprilTags.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            GainControl gainControl = visionPortalAprilTags.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
        try {
            Thread.sleep(20);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
    public double getYellowSampleAngle() {
        ColorBlobLocatorProcessor.Blob yellowBlob = getLargestBlob(yellowColorLocator);
        return getSampleAngle(yellowBlob);
    }
    public double getAllianceSampleAngle() {
        ColorBlobLocatorProcessor.Blob blob = null;
        if (Match.getInstance().getAlliance() == Alliance.Color.RED) {
            blob = getLargestBlob(redColorLocator);
        }
        else {
            blob = getLargestBlob(blueColorLocator);
        }
        return getSampleAngle(blob);
    }

    public void enableBlueLocator(boolean enable) {
        this.visionPortalSamples.setProcessorEnabled(blueColorLocator, enable);
    }
    public void enableRedLocator(boolean enable) {
        this.visionPortalSamples.setProcessorEnabled(blueColorLocator, enable);
    }
    public void enableYellowLocator(boolean enable) {
        this.visionPortalSamples.setProcessorEnabled(blueColorLocator, enable);
    }
    public void stop() {
        giveLightFeedback(null, null, null);
    }
}
