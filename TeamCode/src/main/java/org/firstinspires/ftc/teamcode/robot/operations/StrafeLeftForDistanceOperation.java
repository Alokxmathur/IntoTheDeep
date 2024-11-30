package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.game.Field;

import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class StrafeLeftForDistanceOperation extends DriveTrainOperation {
    private double distance;
    private double speed;

    public StrafeLeftForDistanceOperation(double distance, double speed, String title) {
        super();
        this.distance = distance;
        this.speed = speed;
        this.driveTrain = driveTrain;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "StrafeLeft: %.2f\"@%.2f --%s",
                this.distance/ Field.MM_PER_INCH,
                this.speed,
                this.title);
    }

    public boolean isComplete() {
        if (driveTrain.driveTrainWithinRange()) {
            driveTrain.stop();
            return true;
        }
        return false;
    }

    public double getSpeed() {
        return this.speed;
    }

    public double getDistance() {
        return this.distance;
    }

    @Override
    public void startOperation() {
        driveTrain.handleOperation(this);
    }
}

