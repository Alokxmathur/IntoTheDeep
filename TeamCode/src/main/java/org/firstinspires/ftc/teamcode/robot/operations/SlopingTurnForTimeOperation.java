package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.game.Field;

import java.util.Date;
import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class SlopingTurnForTimeOperation extends DriveTrainOperation {
    protected long time;
    protected double leftSpeed, rightSpeed;

    /**
     * Drive forward for the distance specified at the speed specified
     * @param leftSpeed -1 to 1
     * @param rightSpeed -1 to 1
     * @param title
     */
    public SlopingTurnForTimeOperation(double leftSpeed, double rightSpeed, long time, String title) {
        super();
        this.time = time;
        this.leftSpeed = leftSpeed;
        this.rightSpeed = rightSpeed;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "SlopingTurnForTime: LeftSpeed:%.2f, RightSpeed:%.2f for %d milliseconds--%s",
                this.leftSpeed, this.rightSpeed, this.time,
                this.title);
    }

    public boolean isComplete() {
        Date now = new Date();
        if (now.getTime() - this.getStartTime().getTime() > this.time) {
            driveTrain.stop();
            return true;
        } else {
            return false;
        }
    }

    public double getLeftSpeed() {
        return this.leftSpeed;
    }
    public double getRightSpeed() {
        return this.rightSpeed;
    }
    @Override
    public void startOperation() {
        driveTrain.handleOperation(this);
    }
}

