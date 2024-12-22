package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.DriveTrain;

import java.util.Date;
import java.util.Locale;

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
            return true;
        }
        else {
            double currentBearing =
                    Math.toDegrees(Match.getInstance().getRobot().getPose().heading.toDouble());
            // adjust relative SPEED based on desiredHeading error.
            double bearingError = AngleUnit.normalizeDegrees(Math.toDegrees(heading)
                    - currentBearing);
            double steer = DriveTrain.getSteer(bearingError, DriveTrain.P_DRIVE_COEFFICIENT);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                steer *= -1.0;
            double speedToUse = new Date().getTime() - this.getStartTime().getTime() < 500 ? 0.1 : speed;
            double leftSpeed = speedToUse - steer;
            double rightSpeed = speedToUse + steer;

            // Normalize speeds if either one exceeds +/- 1.0;
            double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > 1.0) {
                leftSpeed /= max;
                rightSpeed /= max;
            }
            Match.log(String.format(Locale.getDefault(), "%.2f vs %.2f, Bearing error: %.2f, Setting power LF:%.2f,LR:%.2f,RF:%.2f,RR%.2f",
                    Math.toDegrees(heading), currentBearing, bearingError, leftSpeed, leftSpeed, rightSpeed, rightSpeed));

            driveTrain.setLeftFrontPower(leftSpeed);
            driveTrain.setLeftBackPower(leftSpeed);
            driveTrain.setRightFrontPower(rightSpeed);
            driveTrain.setRightBackPower(rightSpeed);
            //Match.log(String.format(Locale.getDefault(), "Left speed: %.2f, right: %.2f", leftSpeed, rightSpeed));

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
        driveTrain.handleOperation(this);
    }
}

