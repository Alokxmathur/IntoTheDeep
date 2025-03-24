package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class StrafeLeftToAprilTagOperation extends StrafeRightToAprilTagOperation {

    public StrafeLeftToAprilTagOperation(String title) {
        super(0, title);
    }
    public boolean isComplete() {
        List<AprilTagDetection> aprilTagsSeen = Match.getInstance().getRobot().getVisionPortal().getAprilTags();
        if (!aprilTagsSeen.isEmpty()) {
            driveTrain.stop();
            return true;
        }
        else {
            double currentBearing =
                    Math.toDegrees(Match.getInstance().getRobot().getHeading());
            double bearingError = AngleUnit.normalizeDegrees(Math.toDegrees(this.heading) - currentBearing);
            this.driveTrain.drive(Math.atan2(1, 0), Math.hypot(-RobotConfig.APRIL_TAG_SPEED, 0),
                    -bearingError*PROPORTIONAL_FACTOR);
            Match.log("Correcting bearing from " + currentBearing + " to " + Math.toDegrees(this.heading)
                    + " with rotation of " + -bearingError*PROPORTIONAL_FACTOR);
        }
        return false;
    }
    @Override
    public void startOperation() {
        this.driveTrain.drive(Math.atan2(-RobotConfig.APRIL_TAG_SPEED, 0), Math.hypot(-RobotConfig.APRIL_TAG_SPEED, 0), 0);
    }
}
