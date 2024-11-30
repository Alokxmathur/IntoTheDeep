package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.DriveTrain;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
import java.util.Locale;

/**
 * Drive in the direction specified in degrees, until the specified april tag is seen,
 * at the speed specified
 */
public class DriveInDirectionUntilAprilTagOperation extends DriveInDirectionOperation {

    protected int aprilTagId;
    /**
     * Create an operation to drive in the specified heading
     * Operation completes when the desired april tag is seen or the distance is traveled
     * @param distance - max distance to travel
     * @param aprilTagId - id of april tag to find
     * @param heading - the heading in radians
     * @param speed
     * @param title
     */
    public DriveInDirectionUntilAprilTagOperation(double distance, int aprilTagId, double heading,
                                                  double speed, String title) {
        super(distance, heading, speed, title);
        this.aprilTagId = aprilTagId;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "DriveUntilAprilTagInDirection: Tag:%d, %.2f(%.2f\")@%.2f --%s",
                this.aprilTagId, this.distance, (this.distance / Field.MM_PER_INCH), this.direction,
                this.title);
    }

    public boolean isComplete() {
        List<AprilTagDetection> currentDetections = Match.getInstance().getRobot().getVisionPortal().getAprilTags();
        boolean foundTag = false;
        for (AprilTagDetection detection: currentDetections) {
            if (detection.id == this.aprilTagId) {
                foundTag = true;
                break;
            }
        }
        //we are done if we find the april tag or we have traveled the max distance
        //whichever happens first
        if (foundTag || super.isComplete()) {
            Match.getInstance().getRobot().getDriveTrain().stop();
            return true;
        }

        return false;
    }
}

