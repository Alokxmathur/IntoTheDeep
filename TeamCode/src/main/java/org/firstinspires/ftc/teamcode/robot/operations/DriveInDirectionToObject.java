package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;

import java.util.Locale;

import androidx.annotation.NonNull;

/**
 * Drive in the direction specified in radians, for a maximum of the specified distance
 * or until the distance sensor sees an object closer than a specified object distance,
 * at the speed specified
 */
public class DriveInDirectionToObject extends DriveInDirectionOperation {
    private final double objectDistance;

    /**
     * Create an operation to drive in the specified heading
     * Operation completes when the desired color is seen or the distance is traveled
     * @param distance - max distance to travel
     * @param objectDistance - minimum distance to object
     * @param heading - the heading in radians
     * @param speed - speed at which to move
     * @param title - description of the operation
     */
    public DriveInDirectionToObject(double distance, double objectDistance, double heading,
                                    double speed, String title) {
        super(distance, heading, speed, title);
        this.objectDistance = objectDistance;
    }

    @NonNull
    public String toString() {
        return String.format(Locale.getDefault(), "DriveInDirectionToObject: Max:%.2f(%.2f\"), objectDistance:%.2f(%.2f\")@%.2f --%s",
                this.distance, (this.distance / Field.MM_PER_INCH),
                this.objectDistance, (this.objectDistance / Field.MM_PER_INCH),
                this.direction,
                this.title);
    }

    public boolean isComplete() {
        double distanceToObject = 10;//Match.getInstance().getRobot().getArm().getDistanceToObject();
        //we are done if we find an object within the specified object distance,
        // or we have traveled the max distance
        //whichever happens first
        if (distanceToObject < objectDistance || super.isComplete()) {
            Match.getInstance().getRobot().getDriveTrain().stop();
            Match.log(String.format(Locale.getDefault(), "Distance to object=%.2f vs specified: %.2f",
                    distanceToObject, objectDistance));
            Match.log("Drive train status=" + Match.getInstance().getRobot().getDriveTrain().getStatus());
            return true;
        }

        return false;
    }
}

