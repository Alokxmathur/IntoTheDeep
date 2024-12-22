package org.firstinspires.ftc.teamcode.robot.operations;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.game.Match;

import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class BearingOperation extends FollowTrajectory {
    protected double desiredBearing;

    /**
     * Create a bearing operation
     * @param desiredBearing - desired bearing (radians)
     * @param title
     */
    public BearingOperation(double desiredBearing, String title) {
        super(null, title);
        this.title = title;
        this.desiredBearing = desiredBearing;
    }

    public String toString() {
        return String.format(Locale.getDefault(),"Bearing: %.2f --%s",
                Math.toDegrees(this.desiredBearing), this.title);
    }

    public double getDesiredBearing() {
        return desiredBearing;
    }

    @Override
    public void startOperation() {
        Pose2d currentPose = Match.getInstance().getRobot().getPose();
        Match.log(String.format(Locale.getDefault(), "Starting bearing operation at %.2f,%.2f@%.2f",
                currentPose.position.x, currentPose.position.y, Math.toDegrees(currentPose.heading.toDouble())));
        super.action = driveTrain.actionBuilder(currentPose).turnTo(desiredBearing).build();
        super.startOperation();
    }
}
