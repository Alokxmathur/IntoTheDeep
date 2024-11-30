package org.firstinspires.ftc.teamcode.robot.operations;

import java.util.Date;
import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class DriveForTimeOperation extends DriveTrainOperation {
    private long time;
    private double robotRelativeHeading;
    private double speed;

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public long getTime() {
        return time;
    }

    /**
     * Drive for the specified time
     * @param time - the number of milliseconds
     * @param heading - the heading relative to the robot in radians
     *                This is not the field heading
     * @param speed
     * @param title
     */
    public DriveForTimeOperation(long time, double heading, double speed, String title) {
        super();
        this.time = time;
        this.robotRelativeHeading = heading;
        this.speed = speed;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "DriveForTime: %d@%.2f --%s",
                this.time, this.robotRelativeHeading,
                this.title);
    }

    public boolean isComplete() {
        if (new Date().getTime() > (this.getStartTime().getTime() + getTime())) {
            driveTrain.stop();
            return true;
        } else {
            driveTrain.drive(this.robotRelativeHeading, this.getSpeed(), 0);
            return false;
        }
    }

    public double getSpeed() {
        return this.speed;
    }

    @Override
    public void startOperation() {

    }

}

