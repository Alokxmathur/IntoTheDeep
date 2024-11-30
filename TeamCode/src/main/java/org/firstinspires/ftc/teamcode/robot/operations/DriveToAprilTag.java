package org.firstinspires.ftc.teamcode.robot.operations;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.DriveTrain;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
import java.util.Locale;

/**
 * Drive in the direction specified in degrees, the amount specified in mms at the speed specified
 */
public class DriveToAprilTag extends Operation {

    public static final double SPEED_GAIN  =  0.08  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    public static final double STRAFE_GAIN =  0.04 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    public static final double TURN_GAIN   =  0.03  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    public static final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

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
     * Drive the robot so that it is aligned with a specified april tag id and is within the
     * specified distance from it
     * @param xOffset - how far to the right of the camera the april tag should be (in mms)
     * @param yOffset - how far to the front of the camera the april tag should be (in mms)
     * @param driveTrain
     * @return
     */
    public static boolean driveToAprilTag(double xOffset, double yOffset, DriveTrain driveTrain) {
        double drive = 0, turn = 0, strafe = 0;
        boolean arrived = false;
        AprilTagDetection desiredTag;
        //only do something if we are seeing an april tag
        if ((desiredTag = findTarget()) != null) {
            // Determine x (left-right), y (forward) and Yaw (tag image rotation) error
            // so we can use them to control the robot automatically.
            //xError is how far to the right of the camera, the april tag is - how far right we want to be
            double xError = xOffset/Field.MM_PER_INCH-desiredTag.ftcPose.x;
            //yError is how far the april tag is in front of the camera
            double yError = desiredTag.ftcPose.y - yOffset/Field.MM_PER_INCH;
            //heading is how far the april tag in terms of our heading
            double headingError = desiredTag.ftcPose.bearing;

            double speed = RobotConfig.APRIL_TAG_SPEED;
            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive = Range.clip(yError * SPEED_GAIN, -speed, speed);
            turn = Range.clip(headingError * TURN_GAIN, -speed * .6, speed * .6);
            strafe = Range.clip(-xError * STRAFE_GAIN, -speed, speed);

            /*
            We consider we have arrived if we are within 1 inch of the desired distance,
            within 1 inches of left to right and within 2 degrees of facing the aprilTag
             */
            if (Math.abs(yError) < 1 && Math.abs(xError) < 1 && Math.abs(headingError) < 2) {
                arrived = true;
            }
            else {
                Match.log(String.format(Locale.getDefault(),
                        "Drive to april tag: xError:%.2f, yError:%.2f, heading error: %.2f," +
                                "drive: %.2f, strafe: %.2f, turn: %.2f",
                        xError, yError, headingError,
                        drive, strafe, turn));
            }
        }
        else {
            //if we are not seeing the tag, we say we have arrived as there is no chance we are going to see it
            Match.log("Not seeing any april tags");
            arrived = true;
        }
        if (arrived) {
            driveTrain.stop();
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
    public static AprilTagDetection findTarget() {
        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = Match.getInstance().getRobot().getVisionPortal().getAprilTags();
        //Match.log("Found " + currentDetections.size() + " april tags");
        if (currentDetections.size() > 0) {
            return currentDetections.get(0);
        }
        else {
            Match.log("No april tags to align with");
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
    public static void moveRobot(double forward, double strafe, double rotate, DriveTrain driveTrain) {
        // Calculate wheel powers.
        double leftFrontPower    =  forward -strafe -rotate;
        double rightFrontPower   =  forward +strafe +rotate;
        double leftBackPower     =  forward +strafe -rotate;
        double rightBackPower    =  forward -strafe +rotate;

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
        driveTrain.setLeftRearPower(leftBackPower);
        driveTrain.setRightRearPower(rightBackPower);
    }
}

