package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
import java.util.Locale;

import androidx.annotation.NonNull;

public class StrafeRightToAprilTagOperation extends DriveTrainOperation{
    public static double PROPORTIONAL_FACTOR = .01;
    double heading;

    /**
     * Strafe right in the provided heading until some AprilTag is seen
     * @param heading
     * @param title
     */
    public StrafeRightToAprilTagOperation(double heading, String title) {
        super();
        this.heading = heading;
        this.title = title;
    }

    @NonNull
    public String toString() {
        return String.format(Locale.getDefault(), "StrafeToAprilTag with heading %.2f: --%s",
                Math.toDegrees(this.heading), this.title);
    }

    public boolean isComplete() {
        List<AprilTagDetection> aprilTagsSeen = Match.getInstance().getRobot().getVisionPortal().getAprilTags();
        if (!aprilTagsSeen.isEmpty()) {
             driveTrain.stop();
            return true;
        }
        else {
            double currentBearing =
                    Match.getInstance().getRobot().getHeading();
            //Math.toDegrees(Match.getInstance().getRobot().getPose().getHeading());
            double bearingError = AngleUnit.normalizeDegrees(Math.toDegrees(this.heading) - currentBearing);
            this.driveTrain.drive(Math.atan2(1, 0), Math.hypot(RobotConfig.APRIL_TAG_SPEED, 0),
                    -bearingError*PROPORTIONAL_FACTOR);
            Match.log("Correcting bearing from " + currentBearing + " to " + Math.toDegrees(this.heading)
                + " with rotation of " + -bearingError*PROPORTIONAL_FACTOR);
        }
        return false;
    }
    @Override
    public void startOperation() {
        this.driveTrain.drive(Math.atan2(1, 0), Math.hypot(RobotConfig.APRIL_TAG_SPEED, 0), 0);
    }
}
