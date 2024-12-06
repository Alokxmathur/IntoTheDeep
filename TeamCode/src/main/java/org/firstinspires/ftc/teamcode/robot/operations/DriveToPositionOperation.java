package org.firstinspires.ftc.teamcode.robot.operations;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.Locale;

/**
 * Operation to reach a specified pose.
 *
 * We accomplish this by using the FollowTrajectory class and setting its action on the
 * fly when the operation is started.
 */

public class DriveToPositionOperation extends FollowTrajectory {
    protected Pose2d desiredPose;

    /**
     * Get the robot to the desired pose
     * @param desiredPose
     * @param title
     */
    public DriveToPositionOperation(Pose2d desiredPose, String title) {
        super( null, title);
        this.desiredPose = desiredPose;
        this.title = title;
    }

    public Pose2d getDesiredPose() {
        return this.desiredPose;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "DriveTo: %s --%s",
                this.desiredPose.toString(),
                this.title);
    }

    @Override
    public void startOperation() {
        Robot robot = Match.getInstance().getRobot();
        Pose2d currentPose = robot.getPose();
        action  =
                robot.getDriveTrain().actionBuilder(currentPose)
                        .splineToLinearHeading(desiredPose, 0).build();
    }

    @Override
    public void abortOperation() {
        driveTrain.stop();
    }

}

