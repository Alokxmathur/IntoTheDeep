package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class FollowPathChain extends Operation {
    PathChain pathChain;
    Point endPoint;
    Follower follower;

    double accuracy;
    public FollowPathChain(PathChain pathChain, double accuracy, String title) {
        this.pathChain = pathChain;
        this.endPoint = pathChain.getPath(pathChain.size()-1).getPoint(1);
        this.accuracy = accuracy;
        this.follower = Match.getInstance().getRobot().getFollower();
        this.title = title;
    }

    public FollowPathChain(PathChain pathChain, String title) {
        this(pathChain, 1, title);
    }

    /**
     * Check to see if are within the specified accuracy (in inches) of the end point
     * @return true if we are, false otherwise
     */
    @Override
    public boolean isComplete() {
        follower.update();
        Pose currentPose = follower.getPose();
        //consider completion when both x and y are withing 1 inch of the end point
        boolean complete = Math.abs(currentPose.getX() - endPoint.getX()) <= accuracy
                && Math.abs(currentPose.getY() - endPoint.getY()) <= accuracy;
        if (complete) {
            Match.log("Completed " + title + " at " + currentPose.toString());
        };
        return complete;
    }

    @Override
    public void startOperation() {
        follower.followPath(pathChain);
    }

    @Override
    public void abortOperation() {
    }

    public String toString() {
        return "FollowPathChain: " + title;
    }

    public void setAccuracy(double accuracy) {
        this.accuracy = accuracy;
    }
}
