package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
import java.util.Locale;

public class StrafeRightToAprilTagOperation extends DriveTrainOperation{
    public StrafeRightToAprilTagOperation(String title) {
        super();
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "StrafeToAprilTag: --%s",
                this.title);
    }

    public boolean isComplete() {
        List<AprilTagDetection> aprilTagsSeen = Match.getInstance().getRobot().getVisionPortal().getAprilTags();
        if (aprilTagsSeen.size() > 0) {
             driveTrain.stop();
            return true;
        }
        return false;
    }

    @Override
    public void startOperation() {
        this.driveTrain.drive(Math.atan2(RobotConfig.APRIL_TAG_SPEED, 0), Math.hypot(RobotConfig.APRIL_TAG_SPEED, 0), 0);
    }
}
