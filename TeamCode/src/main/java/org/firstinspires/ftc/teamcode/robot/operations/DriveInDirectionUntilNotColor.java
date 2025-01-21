package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;

import java.util.Locale;

/**
 * Drive in the direction specified in degrees, until the specified april tag is seen,
 * at the speed specified
 */
public class DriveInDirectionUntilNotColor extends DriveInDirectionOperation {

    /**
     * Create an operation to drive in the specified heading
     * Operation completes when the desired april tag is seen or the distance is traveled
     * @param distance - max distance to travel
     * @param heading - the heading in radians
     * @param speed
     * @param title
     */
    public DriveInDirectionUntilNotColor(double distance, double heading,
                                         double speed, String title) {
        super(distance, heading, speed, title);
    }

    public String toString() {
        return String.format(Locale.getDefault(), "DriveUntilColorInDirection: %.2f(%.2f\")@%.2f --%s",
                this.distance, (this.distance / Field.MM_PER_INCH), this.direction,
                this.title);
    }

    public boolean isComplete() {
        boolean foundColor;
        if (Match.getInstance().getAlliance() == Alliance.Color.RED) {
            foundColor = Match.getInstance().getRobot().getColors().red > .07;
        }
        else {
            foundColor = Match.getInstance().getRobot().getColors().blue > .07;
        }
        //we are done if we did not find the color or we have traveled the max distance
        //whichever happens first
        if (!foundColor || super.isComplete()) {
            Match.getInstance().getRobot().getDriveTrain().stop();
            return true;
        }

        return false;
    }
}

