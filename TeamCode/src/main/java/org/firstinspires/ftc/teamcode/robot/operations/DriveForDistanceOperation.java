package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.game.Field;

import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class DriveForDistanceOperation extends DriveTrainOperation {
    protected double distance;
    protected double speed;

    /**
     * Drive forward for the distance specified at the speed specified
     * @param distance - distance to travel in mms
     * @param speed -1 to 1
     * @param title
     */
    public DriveForDistanceOperation(double distance, double speed, String title) {
        super();
        this.distance = distance;
        this.speed = speed;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "Forward: %.2f(%.2f\")@%.2f --%s",
                this.distance, this.distance/ Field.MM_PER_INCH, this.speed,
                this.title);
    }

    public boolean isComplete() {
        if (driveTrain.driveTrainWithinRange()) {
            return true;
        }
        return false;
    }

    public double getSpeed() {
        return this.speed;
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }
    public double getDistance() {
        return this.distance;
    }

    @Override
    public void startOperation() {
        driveTrain.handleOperation(this);
    }
}

