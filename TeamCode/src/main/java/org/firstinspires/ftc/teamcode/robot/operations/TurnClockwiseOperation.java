package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.game.Field;

import java.util.Locale;

/**
 * Turns clockwise the .
 */

public class TurnClockwiseOperation extends DriveForDistanceOperation {

    public TurnClockwiseOperation(double distance, double speed, String title) {
        super(distance, speed, title);
    }

    public String toString() {
        return String.format(Locale.getDefault(), "TurnClockwise: %.2f(%.2f\")@%.2f --%s",
                this.distance, this.distance/ Field.MM_PER_INCH, this.speed,
                this.title);
    }

    @Override
    public void startOperation() {
        driveTrain.handleOperation(this);
    }

    @Override
    public void abortOperation() {
        driveTrain.stop();
    }
}

