package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.DriveTrain;

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
    @Override
    public boolean isComplete() {
        double error = AngleUnit.normalizeDegrees(Math.toDegrees(bearing))
            - AngleUnit.normalizeDegrees(Math.toDegrees((Match.getInstance().getRobot().getPose().getHeading())));
        double speedToUse = Math.max(Math.min(Math.abs(error) * COEFFECIENT, speed), .2);
        DriveTrain driveTrain = Match.getInstance().getRobot().getDriveTrain();
        driveTrain.setLeftFrontPower(-speedToUse);
        driveTrain.setLeftBackPower(-speedToUse);
        driveTrain.setRightFrontPower(speedToUse);
        driveTrain.setRightBackPower(speedToUse);

        return Math.abs(error)  < 4;
    }
}

