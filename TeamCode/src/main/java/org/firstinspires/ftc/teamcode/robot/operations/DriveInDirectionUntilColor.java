package org.firstinspires.ftc.teamcode.robot.operations;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.opencv.core.Mat;

import java.util.List;
import java.util.Locale;

/**
 * Drive in the direction specified in degrees, until the specified april tag is seen,
 * at the speed specified
 */
public class DriveInDirectionUntilColor extends DriveInDirectionOperation {

    /**
     * Create an operation to drive in the specified heading
     * Operation completes when the desired color is seen or the distance is traveled
     * @param distance - max distance to travel
     * @param heading - the heading in radians
     * @param speed
     * @param title
     */
    public DriveInDirectionUntilColor(double distance, double heading,
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
        NormalizedRGBA colors = Match.getInstance().getRobot().getColors();
        if (Match.getInstance().getAlliance() == Alliance.Color.RED) {
            foundColor = colors.red > .07;
        }
        else {
            foundColor = colors.blue > .06;
        }
        //we are done if we find the color or we have traveled the max distance
        //whichever happens first
        if (foundColor || super.isComplete()) {
            Match.getInstance().getRobot().getDriveTrain().stop();
            Match.log(String.format(Locale.getDefault(), "R:%.2f,G:%.2f,B:%.2f,Found color: %s",
                    colors.red, colors.green, colors.blue, "" + foundColor));
            Match.log("Drive train status=" + Match.getInstance().getRobot().getDriveTrain().getStatus());
            return true;
        }

        return false;
    }
}

