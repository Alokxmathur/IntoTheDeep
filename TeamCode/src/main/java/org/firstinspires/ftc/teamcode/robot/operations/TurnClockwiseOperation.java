package org.firstinspires.ftc.teamcode.robot.operations;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.DriveTrain;

import java.util.Locale;

/**
 * Turns clockwise to the desired bearing
 */

public class TurnClockwiseOperation extends Operation {

    double bearing, speed;
    double COEFFECIENT = .01;
    public TurnClockwiseOperation(double bearing, double speed, String title) {
        this.bearing = bearing;
        this.speed = speed;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "TurnClockwise: to %.2f @%.2f --%s",
                Math.toDegrees(this.bearing), this.speed,
                this.title);
    }

    @Override
    public boolean isComplete() {
        double error = AngleUnit.normalizeDegrees(Math.toDegrees((Match.getInstance().getRobot().getPose().getHeading()))
                - AngleUnit.normalizeDegrees(Math.toDegrees(bearing)));
        double speedToUse = Math.max(Math.min(Math.abs(error) * COEFFECIENT, speed), .2);
        DriveTrain driveTrain = Match.getInstance().getRobot().getDriveTrain();
        driveTrain.setLeftFrontPower(speedToUse);
        driveTrain.setLeftBackPower(speedToUse);
        driveTrain.setRightFrontPower(-speedToUse);
        driveTrain.setRightBackPower(-speedToUse);

        return Math.abs(error)  < 4;
    }

    @Override
    public void startOperation() {
    }

    @Override
    public void abortOperation() {
        Match.getInstance().getRobot().getDriveTrain().stop();
    }
}

