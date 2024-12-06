package org.firstinspires.ftc.teamcode.robot.operations;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.opmodes.drivercontrolled.RobotAutoDriveToAprilTagOmni;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.DriveTrain;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
import java.util.Locale;

/**
 * Drive in the direction specified in degrees, the amount specified in mms at the speed specified
 */
public class DriveToAprilTag extends Operation {
    protected double xOffset, yOffset;
    protected DriveTrain driveTrain;

    AprilTagDetection desiredTag;
    /**
     * Create an operation to drive in the specified heading
     * @param xOffset - how far to the left of the april tag we should be
     * @param yOffset - how far from the april tag we should be
     * @param title
     */
    public DriveToAprilTag(double xOffset, double yOffset,  String title) {
        this.xOffset = xOffset;
        this.yOffset = yOffset;
        this.driveTrain = Match.getInstance().getRobot().getDriveTrain();
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "DriveToAprilTag: X:%.2f(%.2f\"), Y:%.2f(%.2f\") --%s",
                this.xOffset, (this.xOffset / Field.MM_PER_INCH),
                this.yOffset, (this.yOffset / Field.MM_PER_INCH),
                this.title);
    }

    public boolean isComplete() {
        return driveToAprilTag(xOffset, yOffset, driveTrain);
    }

    /**
     * Drive the robot so that it is aligned with a specified an april tag and is within the
     * specified distance from it and is the specified left or right distance to it
     * @param xOffset - how far to the right of the camera the april tag should be (in mms)
     * @param yOffset - how far to the front of the camera the april tag should be (in mms)
     * @param driveTrain
     * @return
     */
    public static boolean driveToAprilTag(double xOffset, double yOffset, DriveTrain driveTrain) {
        double drive = 0, turn = 0, strafe = 0;
        boolean arrived = false;
        AprilTagDetection desiredTag;
        double rangeError = 0, headingError = 0, yawError = 0;
        //only do something if we are seeing an april tag
        if ((desiredTag = findTarget(14)) != null) {
            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            rangeError      = (desiredTag.ftcPose.range - (yOffset/Field.MM_PER_INCH));
            headingError    = desiredTag.ftcPose.bearing;
            yawError        = desiredTag.ftcPose.yaw;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive  = Range.clip(rangeError * RobotAutoDriveToAprilTagOmni.SPEED_GAIN,
                    -RobotAutoDriveToAprilTagOmni.MAX_AUTO_SPEED,
                    RobotAutoDriveToAprilTagOmni.MAX_AUTO_SPEED);
            turn   = Range.clip(headingError * RobotAutoDriveToAprilTagOmni.TURN_GAIN,
                    -RobotAutoDriveToAprilTagOmni.MAX_AUTO_TURN, RobotAutoDriveToAprilTagOmni.MAX_AUTO_TURN) ;
            strafe = Range.clip(-yawError * RobotAutoDriveToAprilTagOmni.STRAFE_GAIN,
                    -RobotAutoDriveToAprilTagOmni.MAX_AUTO_STRAFE, RobotAutoDriveToAprilTagOmni.MAX_AUTO_STRAFE);

            /*
            We consider we have arrived if we are within 1 inch of the desired distance,
            and within 2 degrees of facing the aprilTag
             */
            if (rangeError < 1 ) {
                arrived = true;
            }
            Match.log(String.format(Locale.getDefault(),
                    "Drive to april tag: yError:%.2f, heading error: %.2f," +
                            "drive: %.2f, strafe: %.2f, turn: %.2f",
                    rangeError, headingError,
                    drive, strafe, turn));
        }
        else {
            //if we are not seeing the tag, we say we have arrived as there is no chance we are going to see it
            Match.log("Not seeing any april tags");
            //arrived = true;
        }
        if (arrived) {
            driveTrain.stop();
            Match.log(String.format(Locale.getDefault(),
                    "Drive to april tag completed: yError:%.2f, heading error: %.2f," +
                            "drive: %.2f, strafe: %.2f, turn: %.2f",
                    rangeError, headingError,
                    drive, strafe, turn));
        }
        else {
            moveRobot(drive, strafe, turn, driveTrain);
        }
        return arrived;

    }

    @Override
    public void startOperation() {
    }

    @Override
    public void abortOperation() {
        driveTrain.stop();
    }

    public double getxOffset() {
        return this.xOffset;
    }

    /**
     * Return an april tag detection if it is seen. Return null if it is not
     * @return
     */
    public static AprilTagDetection findTarget(int targetId) {
        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = Match.getInstance().getRobot().getVisionPortal().getAprilTags();
        //Match.log("Found " + currentDetections.size() + " april tags");
        for (AprilTagDetection detection: currentDetections) {
            if (detection.id == targetId) {
                return detection;
            }
        }
        return null;
    }
    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public static void moveRobot(double x, double y, double yaw, DriveTrain driveTrain) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        driveTrain.setLeftFrontPower(leftFrontPower);
        driveTrain.setRightFrontPower(rightFrontPower);
        driveTrain.setLeftBackPower(leftBackPower);
        driveTrain.setRightBackPower(rightBackPower);
    }
}

