package org.firstinspires.ftc.teamcode.robot.components.vision.detector;

import org.firstinspires.ftc.teamcode.game.Match;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * A class representing types of objects on the competition field
 * Currently the object differentiation is done based on HSV ranges
 * <p>
 * The class contains a list of object of the type found
 */
public class DetectableObject {
    ObjectDetector.ObjectType type;
    ObjectDetector.HsvBounds[] hsvBounds;
    List<MatOfPoint> foundObjects = new ArrayList<>();

    double largestArea;
    double width;
    double height;
    int largestAreaIndex;
    String shortName;

    boolean disabled = true;

    public DetectableObject(ObjectDetector.ObjectType type, String shortName, ObjectDetector.HsvBounds[] hsvBounds, double width, double height) {
        this.type = type;
        this.setShortName(shortName);
        this.hsvBounds = hsvBounds;
        this.width = width;
        this.height = height;
    }

    public String getShortName() {
        if (shortName == null) {
            return this.type.toString();
        }
        return shortName;
    }
    public void setShortName(String shortName) {
        this.shortName = shortName;
    }

    public ObjectDetector.ObjectType getType() {
        return this.type;
    }

    public double getWidth() {
        return width;
    }

    public double getHeight() {
        return height;
    }

    public ObjectDetector.HsvBounds[] getHsvBounds() {
        return hsvBounds;
    }

    public void setHsvBounds(ObjectDetector.HsvBounds hsvBounds[]) {
        this.hsvBounds = hsvBounds;
    }

    public List<MatOfPoint> getFoundObjects() {
        return this.foundObjects;
    }

    public void clearFoundObjects() {
        if (this.foundObjects != null) {
            for (MatOfPoint contour: this.foundObjects) {
                try {
                    contour.release();
                }
                catch (Throwable e) {}
            }
        }
        this.foundObjects = new ArrayList<>();
        largestArea = 0;
    }

    /**
     * Add an object to the list of the objects of this type
     * If the area passed is greater than the largest area already found, the new object passed is
     * considered the largest object and the mean of it considered the mean of the largest object
     *
     * @param objectFound
     * @param area        - are of the object found
     */
    public void addFoundObject(MatOfPoint objectFound, double area) {
        this.foundObjects.add(objectFound);
        if (area > largestArea) {
            largestAreaIndex = this.foundObjects.size() - 1;
            largestArea = area;
            //Match.log("Setting largest area of " + type + " to be " + area);
        }
    }

    /**
     * Returns the largest object seen of this detectable object type
     *
     * @return
     */
    public MatOfPoint getLargestObject() {
        if (largestAreaIndex < this.foundObjects.size()) {
            return this.foundObjects.get(largestAreaIndex);
        } else {
            return null;
        }
    }

    /**
     * Returns the y position of the middle of the largest object
     *
     * @return
     */
    public int getYPositionOfLargestObject() {
        MatOfPoint largestObject = getLargestObject();
        if (largestObject != null) {
            try {
                Rect boundingRectangle = Imgproc.boundingRect(largestObject);
                return boundingRectangle.y + boundingRectangle.height / 2;
            }
            catch (Throwable e){}
        }
        return -1;
    }

    /**
     * Returns the x position of the center of the largest object
     *
     * @return
     */
    public int getXPositionOfLargestObject() {
        MatOfPoint largestObject = getLargestObject();
        if (largestObject != null) {
            try {
                Rect boundingRectangle = Imgproc.boundingRect(largestObject);
                return boundingRectangle.x + boundingRectangle.width / 2;
            }
            catch (Throwable e) {
            }
        }
        return -1;
    }

    /**
     * Returns the width of the largest object
     *
     * @return
     */
    public double getWidthOfLargestObject() {
        RotatedRect rotatedRectangle = getRotatedRectangleOfLargestObject();
        if (rotatedRectangle != null) {
            return rotatedRectangle.size.height;
        } else {
            return -1;
        }
    }

    /**
     * Returns the height of the largest object
     *
     * @return
     */
    public double getHeightOfLargestObject() {
        RotatedRect rotatedRectangle = getRotatedRectangleOfLargestObject();
        if (rotatedRectangle != null) {
            return rotatedRectangle.size.width;
        } else {
            return -1;
        }
    }

    /**
     * Return the bounding rectangle that fits the largest object
     * The rectangle is always aligned with the x and y axes
     *
     * @return
     */
    public Rect getBoundingRectangleOfLargestObject() {
        MatOfPoint largestObject = getLargestObject();
        if (largestObject != null) {
            try {
                return Imgproc.boundingRect(largestObject);
            }
            catch (Throwable e) {
                return null;
            }
        } else {
            return null;
        }
    }

    /**
     * Return the rectangle that fits the largest object
     * The rectangle could be at an angle to the x and y axes
     *
     * @return
     */
    public RotatedRect getRotatedRectangleOfLargestObject() {
        MatOfPoint largestObject = getLargestObject();
        if (largestObject != null) {
            try {
                return Imgproc.minAreaRect(new MatOfPoint2f(largestObject.toArray()));
            }
            catch (Throwable e) {
                return null;
            }
        } else {
            return null;
        }
    }

    public double getLargestArea() {
        return largestArea;
    }

    /**
     * Returns if the object detection is disabled for this object type
     */
    public boolean isDisabled() {
        return disabled;
    }

    public void disable() {
        this.disabled = true;
    }
    public void enable() {
        this.disabled = false;
    }
    public void toggleDisabled() {
        this.disabled = !this.disabled;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "Count:%d,Largest@%d,%d,area:%.0f, placement: %s",
                getFoundObjects().size(),
                getXPositionOfLargestObject(),
                getYPositionOfLargestObject(),
                largestArea,
                getXPositionOfLargestObject() > 1200 ? "Right" : (getXPositionOfLargestObject() > 600 ? "Middle" : "Left"));
    }
}
