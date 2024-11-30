package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.game.Field;
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
     * @param driveTrain
     * @param title
     */
    public StrafeLeftForDistanceWithHeadingOperation(double distance, double heading, double speed, DriveTrain driveTrain, String title) {
        super();
        this.distance = distance;
        this.heading = heading;
        this.speed = speed;
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
        else {
            // adjust relative SPEED based on desiredHeading error.
            double bearingError = AngleUnit.normalizeDegrees(Math.toDegrees(heading) - Math.toDegrees(driveTrain.getExternalHeading()));
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

            driveTrain.setLeftFrontPower(leftSpeed);
            driveTrain.setLeftRearPower(leftSpeed);
            driveTrain.setRightFrontPower(rightSpeed);
            driveTrain.setRightRearPower(rightSpeed);
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

