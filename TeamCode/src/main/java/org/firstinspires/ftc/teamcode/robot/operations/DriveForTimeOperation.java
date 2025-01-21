package org.firstinspires.ftc.teamcode.robot.operations;

import java.util.Date;
import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class DriveForTimeOperation extends DriveTrainOperation {
    private long time;
    private double robotRelativeHeading;
    private double leftSpeed, rightSpeed;

    public void setLeftSpeed(double leftSpeed) {
        this.leftSpeed = leftSpeed;
    }

    public long getTime() {
        return time;
    }

    /**
     * Drive for the specified time
     * @param time - the number of milliseconds
     * @param heading - the heading relative to the robot in radians
     *                This is not the field heading
     * @param leftSpeed
     * @param rightSpeed
     * @param title
     */
    public DriveForTimeOperation(long time, double leftSpeed, double rightSpeed, String title) {
        super();
        this.time = time;
        this.leftSpeed = leftSpeed;
        this.rightSpeed = rightSpeed;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "DriveForTime: %d@Left:%.2f,Right:%.2f --%s",
                this.time, this.leftSpeed, this.rightSpeed,
                this.title);
    }

    public boolean isComplete() {
        if (new Date().getTime() > (this.getStartTime().getTime() + getTime())) {
            driveTrain.stop();
            return true;
        }
        return false;
    }

    @Override
    public void startOperation() {
        this.driveTrain.setLeftBackPower(leftSpeed);
        this.driveTrain.setLeftFrontPower(leftSpeed);
        this.driveTrain.setRightBackPower(rightSpeed);
        this.driveTrain.setRightFrontPower(rightSpeed);
    }

}

