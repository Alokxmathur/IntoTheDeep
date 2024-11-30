package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
import java.util.Locale;

public class StrafeRightToAprilTagOperation extends DriveTrainOperation{
    int desiredAprilTag;

    public StrafeRightToAprilTagOperation(int desiredAprilTag, String title) {
        super();
        this.desiredAprilTag = desiredAprilTag;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "StrafeToAprilTag: Tag:%d --%s",
                this.desiredAprilTag,
                this.title);
    }

    public boolean isComplete() {
        List<AprilTagDetection> aprilTagsSeen = Match.getInstance().getRobot().getVisionPortal().getAprilTags();
        for (AprilTagDetection aprilTagDetection : aprilTagsSeen) {
            if (aprilTagDetection.id == desiredAprilTag) {
                driveTrain.stop();
                return true;
            }
        }
        return false;
    }

    @Override
    public void startOperation() {
        this.driveTrain.drive(Math.atan2(RobotConfig.APRIL_TAG_SPEED, 0), Math.hypot(RobotConfig.APRIL_TAG_SPEED, 0), 0);
    }
}
