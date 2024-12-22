package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.opencv.core.Mat;

public class FollowPath extends Operation {
    Path path;
    Follower follower;
    public FollowPath(Path path, String title) {
        this.path = path;
        this.follower = Match.getInstance().getRobot().getFollower();
        this.title = title;
    }
    @Override
    public boolean isComplete() {
        follower.update();
        //follower.followPath(path, true);
        boolean isBusy = follower.isBusy();
        Match.log("Follower, busy=" + isBusy + " at " + follower.getPose().toString());
        return !isBusy;
    }

    @Override
    public void startOperation() {
        follower.followPath(path, true);
    }

    @Override
    public void abortOperation() {
    }

    public String toString() {
        return "FollowPath: " + title;
    }
}
