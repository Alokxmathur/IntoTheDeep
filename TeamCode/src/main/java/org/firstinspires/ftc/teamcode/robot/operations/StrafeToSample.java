package org.firstinspires.ftc.teamcode.robot.operations;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.opmodes.drivercontrolled.RobotAutoDriveToAprilTagOmni;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.DriveTrain;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.opencv.core.RotatedRect;

import java.util.List;
import java.util.Locale;

public class StrafeToSample extends DriveTrainOperation {

    public enum SampleType {
        Red, Yellow, Blue
    }
    SampleType sampleType;
    public StrafeToSample(SampleType sampleType, String title) {
        super();
        this.sampleType = sampleType;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "StrafeTo %s Sample: --%s",
                this.sampleType, this.title);
    }

    public boolean isComplete() {
        ColorBlobLocatorProcessor.Blob largestObject = null;
        switch (sampleType) {
            case Red: {
                largestObject = Match.getInstance().getRobot().getVisionPortal().getRedObject();
                break;
            }
            case Blue: {
                largestObject = Match.getInstance().getRobot().getVisionPortal().getBlueObject();
                break;
            }
            case Yellow: {
                largestObject = Match.getInstance().getRobot().getVisionPortal().getYellowObject();
                break;
            }
        }
        return strafeToObject(largestObject, driveTrain);
    }

    @Override
    public void startOperation() {
    }

    public static boolean strafeToObject(ColorBlobLocatorProcessor.Blob object, DriveTrain driveTrain) {
        double drive = 0, turn = 0, strafe = 0;
        boolean arrived = false;
        AprilTagDetection desiredTag;
        double rangeError = 0, headingError = 0, yawError = 0;
        //only do something if we are seeing an april tag
        if (object != null) {
            RotatedRect box = object.getBoxFit();
            if (box.center.x > 375) {
                driveTrain.drive(Math.atan2(-1, 0), -.3, 0);
            } else if (box.center.x < 325) {
                driveTrain.drive(Math.atan2(1, 0), .3, 0);
            } else {
                driveTrain.stop();
                return true;
            }
            return false;
        }
        else {
            return true;
        }
    }

}
