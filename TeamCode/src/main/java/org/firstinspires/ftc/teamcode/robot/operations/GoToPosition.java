package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutonomousHelper;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutonomousV2;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class GoToPosition extends Operation {
    PathChain pathChain;
    Point endPoint;
    Follower follower;

    Pose pose;

    double accuracy;
    public GoToPosition(Pose pose, double accuracy, String title) {
        this.pose = pose;
        this.accuracy = accuracy;
        this.follower = Match.getInstance().getRobot().getFollower();
        this.title = title;
    }

    public GoToPosition(Pose pose, String title) {
        this(pose, 1, title);
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
        Pose currentPose = Match.getInstance().getRobot().getPose();
        this.pathChain = new PathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(currentPose.getX(),
                                        currentPose.getY(),
                                        Point.CARTESIAN),
                                new Point(90, 114, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(currentPose.getHeading(), pose.getHeading())
        .build();
        this.endPoint = pathChain.getPath(pathChain.size()-1).getPoint(1);
        follower.followPath(pathChain, true);
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
