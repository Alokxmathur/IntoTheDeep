package org.firstinspires.ftc.teamcode.game;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutonomousHelper;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.Autonomous;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import java.util.Locale;

/**
 * Created by Silver Titans on 9/16/17.
 */
public class Field {
    public static final float MM_PER_INCH = 25.4f;
    public static final float TILE_WIDTH = 24 * MM_PER_INCH;
    public static volatile boolean initialized = true;
    public static final Object mutex = new Object();

    public enum StartingPosition {
        Left, Right, NotSelected
    }
    public enum SpikePosition {
        Left, Middle, Right, NotSeen
    }
    public void init(Alliance.Color alliance, StartingPosition startingPosition) {
    }

    public static boolean isNotInitialized() {
        synchronized (mutex) {
            return !initialized;
        }
    }

    public static final PathChain redSamplesPathChain = new PathBuilder()
            .addPath(
                    // Line 1 - get to top of sample 1
                    new BezierCurve(
                            new Point(
                                    AutonomousHelper.redLeftStartingPose.getX() - (Autonomous.DISTANCE_TO_SUBMERSIBLE/Field.MM_PER_INCH),
                                    AutonomousHelper.redRightStartingPose.getY(), Point.CARTESIAN),
                            new Point(140, 130, Point.CARTESIAN),
                            new Point(80.814, 93.477, Point.CARTESIAN),
                            new Point(80, 115, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .addPath(
                    // Line 2 - push sample 1
                    new BezierLine(
                            new Point(80, 115, Point.CARTESIAN),
                            new Point(114, 115, Point.CARTESIAN)
                    )
            )
            .setTangentHeadingInterpolation()
            .addPath(
                    // Line 3 - return from pushing first sample
                    new BezierCurve(
                            new Point(114, 115, Point.CARTESIAN),
                            new Point(73.030, 120, Point.CARTESIAN),
                            new Point(80, 124, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .addPath(
                    // Line 2 - push sample 2
                    new BezierLine(
                            new Point(80, 124, Point.CARTESIAN),
                            new Point(116, 124, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .setPathEndHeadingConstraint(Math.toRadians(5))
            .setPathEndTValueConstraint(.8)
            .setPathEndTimeoutConstraint(100)
            .build();

            public static String poseToString(Pose pose) {
                return String.format(Locale.getDefault(), "%.2f,%.2f@%.2f",
                        pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
            }
        public static String pointToString(Point point) {
            return String.format(Locale.getDefault(), "%.2f,%.2f",
                    point.getX(), point.getY());
        }
}