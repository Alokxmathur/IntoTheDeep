package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.game.Field;

import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class TurnAntiClockwiseOperation extends TurnClockwiseOperation {

    public TurnAntiClockwiseOperation(double distance, double speed, String title) {
        super(distance, speed, title);
    }

    public String toString() {
        return String.format(Locale.getDefault(), "TurnAnti: %.2f(%.2f\")@%.2f --%s",
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

