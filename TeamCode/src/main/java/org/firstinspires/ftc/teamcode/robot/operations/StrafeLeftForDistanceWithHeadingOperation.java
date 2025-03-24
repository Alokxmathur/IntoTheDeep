package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.DriveTrain;

import java.util.Date;
import java.util.Locale;

import static org.firstinspires.ftc.teamcode.robot.operations.StrafeRightToAprilTagOperation.PROPORTIONAL_FACTOR;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class StrafeLeftForDistanceWithHeadingOperation extends DriveTrainOperation {
    private double distance;
    private double speed;
    private double heading;

    /**
     * Create a Strafe Left maintaining heading operation
     * @param distance - in mm
     * @param heading - in radians
     * @param speed
     * @param title
     */
    public StrafeLeftForDistanceWithHeadingOperation(double distance, double heading, double speed, String title) {
        super();
        this.distance = distance;
        this.heading = heading;
        this.speed = speed;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "StrafeLeft: %.2f\",H:%.2f,@%.2f --%s",
                this.distance/ Field.MM_PER_INCH,
                Math.toDegrees(this.heading),
                this.speed,
                this.title);
    }

    public boolean isComplete() {
        if (driveTrain.driveTrainWithinRange()) {
            driveTrain.stop();
            Match.log("Ending strafe left with heading at " + Field.poseToString(Match.getInstance().getRobot().getPose()));
            return true;
        }
        else {
            double currentBearing =
                    Match.getInstance().getRobot().getHeading();
            //Math.toDegrees(Match.getInstance().getRobot().getPose().getHeading());
            double bearingError = AngleUnit.normalizeDegrees(Math.toDegrees(this.heading) - currentBearing);
            this.driveTrain.drive(-Math.atan2(1, 0), Math.hypot(RobotConfig.APRIL_TAG_SPEED*2, 0),
                    -bearingError*PROPORTIONAL_FACTOR);
            Match.log("Correcting bearing from " + currentBearing + " to " + Math.toDegrees(this.heading)
                    + " with rotation of " + -bearingError*PROPORTIONAL_FACTOR);
            return false;
        }
    }

    public double getSpeed() {
        return this.speed;
    }

    public double getDistance() {
        return this.distance;
    }

    @Override
    public void startOperation() {
        Match.log("Starting strafe left with heading at " + Field.poseToString(Match.getInstance().getRobot().getPose()));
        driveTrain.handleOperation(this);
    }
}

