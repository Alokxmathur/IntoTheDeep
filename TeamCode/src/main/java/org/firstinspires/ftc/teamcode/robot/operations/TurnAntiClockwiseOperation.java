package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;

import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class TurnAntiClockwiseOperation extends TurnClockwiseOperation {

    public TurnAntiClockwiseOperation(double bearing, double speed, String title) {
        super(bearing, speed, title);
    }

    public String toString() {
        return String.format(Locale.getDefault(), "TurnAnti: to %.2f@%.2f --%s",
                Math.toDegrees(bearing), this.speed,
                this.title);
    }

    @Override
    public void startOperation() {
        Match.getInstance().getRobot().getDriveTrain().drive(0, speed, -1);
    }
}

