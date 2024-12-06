package org.firstinspires.ftc.teamcode.robot.operations;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.game.Match;

import java.util.Locale;

/**
 * An operation to follow the specified road-runner trajectory
 */

public class FollowTrajectory extends DriveTrainOperation {
    protected Action action;
    public FollowTrajectory(Action action, String title) {
        super();
        this.action = action;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "FollowTrajectory: %s --%s",
                action.toString(),
                this.title);
    }

    public boolean isComplete() {
        boolean shouldRunAgain = action.run(new TelemetryPacket());
        Pose2d currentPose = Match.getInstance().getRobot().getPose();
        if (shouldRunAgain) {
            Match.log(String.format("Continuing trajectory %s, at %.2f,%.2f@%.2f",
                    this.title,
                    currentPose.position.x,
                    currentPose.position.y,
                    Math.toDegrees(currentPose.heading.toDouble())));
            return false;
        }
        else {
            Match.log(String.format("Finished trajectory %s, at %.2f,%.2f@%.2f",
                    this.title,
                    currentPose.position.x,
                    currentPose.position.y,
                    Math.toDegrees(currentPose.heading.toDouble())));
            return true;
        }
    }

    @Override
    public void startOperation() {

    }
}

