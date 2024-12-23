package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.operations.ArmOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DriveInDirectionOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DriveToAprilTag;
import org.firstinspires.ftc.teamcode.robot.operations.FollowPath;
import org.firstinspires.ftc.teamcode.robot.operations.State;
import org.firstinspires.ftc.teamcode.robot.operations.StrafeLeftForDistanceOperation;
import org.firstinspires.ftc.teamcode.robot.operations.StrafeRightForDistanceOperation;
import org.firstinspires.ftc.teamcode.robot.operations.StrafeRightToAprilTagOperation;
import org.firstinspires.ftc.teamcode.robot.operations.TurnClockwiseOperation;

/**
 * This is PedroPathing version of the IntoTheDeep autonomous
 * It delivers loaded specimen onto the high chamber, then
 * pushes 2 samples into the observation zone, then
 * picks up specimen made by human player from the wall, then
 * delivers the picked up specimen,
 * goes back to the observation zone to pick up the next specimen placed by human player
 * delivers the second specimen to the high chamber
 * parks in the observation zone
 */
public abstract class AutonomousPedro extends AutonomousHelper {
    private final Pose startPose = new Pose(135, 80, Math.toRadians(180));
    private final Pose firstSpecimenScoringPose = new Pose(125, 80, Math.toRadians(180));
    private final Point interimPoint =  new Point(108.000, 108.000, Point.CARTESIAN);

    public static final double DISTANCE_TO_PUSH_SAMPLES = 44.0 * Field.MM_PER_INCH;
    double DISTANCE_TO_SUBMERSIBLE = 15.0 * Field.MM_PER_INCH;
    double RETRACTION_FROM_WALL = 10 * Field.MM_PER_INCH;
    @Override
    public void start() {
        super.start();
        Match.getInstance().getRobot().getFollower().setPose(startPose);
        /**
         * State to deliver specimen to the high chamber
         * This is done by
         * 1. raising and extending the arm for high chamber,
         * 2. moving towards the submersible - this should put the specimen just above the high chamber
         * 3. moving the arm down so specimen snaps onto high chamber
         * 4. moving arm to the release position so it does not get stuck on the chambers
         * 5. retracting from the submersible for latter operations
         */
        State state = new State("Deliver Specimen to high chamber");
        Path reachSubmersiblePath = new Path(new BezierLine(new Point(startPose), new Point(firstSpecimenScoringPose)));
        reachSubmersiblePath.setLinearHeadingInterpolation(startPose.getHeading(), firstSpecimenScoringPose.getHeading(),
                .8);
        state.addPrimaryOperation(new FollowPath(reachSubmersiblePath, "Reach submersible"));

        /*
        state.addSecondaryOperation(new ArmOperation(ArmOperation.Type.Level, "Level Arm"));
        state.addSecondaryOperation(
                new ArmOperation(ArmOperation.Type.High_Chamber, "Raise arm for high chamber"));
        state.addSecondaryOperation(
                new ArmOperation(ArmOperation.Type.High_Chamber_Deposit, "Deposit on high chamber"));
        state.addSecondaryOperation(
                new ArmOperation(ArmOperation.Type.High_Chamber_Release, "Release specimen"));

         */
        Path reachInterimPointPath =
                new Path(
                    new BezierCurve(
                            new Point(125.000, 80.000, Point.CARTESIAN),
                            new Point(130.000, 100.000, Point.CARTESIAN),
                            new Point(120.000, 110.000, Point.CARTESIAN),
                            interimPoint
                    )
                );
        reachInterimPointPath.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0), 0.8);
        state.addPrimaryOperation(new FollowPath(reachInterimPointPath, "Reach interim point"));

        Path reachSamplePath =
                new Path(
                        new BezierCurve(
                                new Point(Match.getInstance().getRobot().getFollower().getPose()),
                                //new Point(108.000, 108.000, Point.CARTESIAN),
                                new Point(69.659, 106.549, Point.CARTESIAN),
                                new Point(76, 120.000, Point.CARTESIAN)
                        )
                );
        reachSamplePath.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
        state.addPrimaryOperation(new FollowPath(reachSamplePath, "Reach sample point"));

        states.add(state);

        /**
         * State to reach a position above the samples so they can be pushed into the observation zone
         * This is done by:
         * 1. Rotating right so the camera can see the april tag on the wall to the right of the human player
         * 2. Getting the robot centered on the second tile in front of human player and second from right wall
         *      we do this by using the camera and the april tag
         * 3. Rotating towards the observation zone
         * 4. Moving backwards away from the observation zone
         *
         * We also raise the arm so it is almost vertical as we do not want it to interfere with movements
         */
        state = new State("Reach zone to push samples");
        state.addPrimaryOperation(new TurnClockwiseOperation(Math.toRadians(-90), 1, "Face april tag"));
        state.addPrimaryOperation(new DriveToAprilTag(0, 28*Field.MM_PER_INCH, "Get to april tag"));
        state.addPrimaryOperation(new TurnClockwiseOperation(Math.toRadians(180), 1, "Face wall"));
        state.addPrimaryOperation(new DriveInDirectionOperation(-1.5*Field.TILE_WIDTH, Math.toRadians(180),
                RobotConfig.CAUTIOUS_SPEED, "Move to top of samples"));
        state.addPrimaryOperation(new StrafeLeftForDistanceOperation(
                14*Field.MM_PER_INCH, RobotConfig.CAUTIOUS_SPEED, "Strafe to get on sample 1"));
        state.addSecondaryOperation(new ArmOperation(ArmOperation.Type.Level, "Get arm level"));
        //states.add(state);

        /**
         * State to push sample 1 (left most) into the observation zone
         * This is done by:
         * 1. Strafing left so the front of the robot can push the sample
         * 2. Moving towards the observation zone
         */

        state = new State("Push sample 1");
        state.addPrimaryOperation(
                new DriveInDirectionOperation(DISTANCE_TO_PUSH_SAMPLES, Math.toRadians(180), RobotConfig.CAUTIOUS_SPEED,
                        "Push sample 1 into observation zone"));
        state.addPrimaryOperation(new DriveInDirectionOperation(
                -DISTANCE_TO_PUSH_SAMPLES, Math.toRadians(180), RobotConfig.CAUTIOUS_SPEED,
                "Retract from observation zone"));
        //states.add(state);

        /**
         * State to push sample 2 (middle) into the observation zone
         * This is done by:
         * 1. Going backwards to we are beyond the second sample
         * 2. Strafing left so the front of the robot can push the sample
         * 3. Moving towards the observation zone
         */
        state = new State("Push sample 2");
        state.addPrimaryOperation(new StrafeLeftForDistanceOperation(
                9*Field.MM_PER_INCH, RobotConfig.CAUTIOUS_SPEED, "Strafe to get on sample 2"));
        state.addPrimaryOperation(
                new DriveInDirectionOperation(DISTANCE_TO_PUSH_SAMPLES, Math.toRadians(180), RobotConfig.CAUTIOUS_SPEED/2,
                        "Push sample 2 into observation zone"));
        state.addSecondaryOperation(new ArmOperation(ArmOperation.Type.Specimen_Intake, "Get to specimen intake level"));
        state.addPrimaryOperation(new ArmOperation(ArmOperation.Type.Hold, "Grab specimen"));
        //states.add(state);

        /**
         * State to deliver specimen collected from observation zone
         * This is done by:
         * 1. Retracting a little to clear the wall
         * 2. Strafing right until we see the april tag on wall closest to players
         * 3. Rotating to face the submersible
         * 4. Moving towards the submersible
         * 5. moving the arm down so specimen snaps onto high chamber
         * 6. moving arm to the release position so it does not get stuck on the chambers
         * 7. retracting from the submersible for latter operations
         */
        state = new State("Deliver first specimen from observation zone");
        state.addPrimaryOperation(
                new DriveInDirectionOperation(-2*Field.MM_PER_INCH, Math.toRadians(180), RobotConfig.CAUTIOUS_SPEED, "Retract a bit"));
        state.addPrimaryOperation(
                new StrafeRightToAprilTagOperation( "Strafe right to reach april tag"));
        state.addPrimaryOperation(new DriveToAprilTag(Math.toRadians(0), 18*Field.MM_PER_INCH, "Align with april tag"));
        state.addPrimaryOperation(new TurnClockwiseOperation(Math.toRadians(0), 1, "Rotate towards submersible"));

        state.addSecondaryOperation(new ArmOperation(ArmOperation.Type.Level, "Retract"));
        //states.add(state);

        /**
         * Deposit second specimen
         */
        state = new State("Deposit second specimen");
        state.addPrimaryOperation(
                new ArmOperation(ArmOperation.Type.High_Chamber, "Raise to high chamber"));
        state.addPrimaryOperation(
                new ArmOperation(ArmOperation.Type.High_Chamber_Deposit, "Deposit on high chamber"));
        state.addPrimaryOperation(
                new ArmOperation(ArmOperation.Type.High_Chamber_Release, "Release specimen"));
        //states.add(state);

        /**
         * State to reach observation zone
         * This is done by
         * 1. Strafing right
         * 2. Moving towards the wall
         */
        state = new State("Reach observation zone");
        state.addPrimaryOperation(
                new DriveInDirectionOperation(-6*Field.MM_PER_INCH, 0, RobotConfig.CAUTIOUS_SPEED, "Retract a bit"));
        state.addPrimaryOperation(
                new StrafeRightForDistanceOperation(2*Field.TILE_WIDTH, RobotConfig.CAUTIOUS_SPEED, "Move toward observation zone"));
        //states.add(state);
    }
    @Override
    public void init() {
        super.init(telemetry, Alliance.Color.RED, Field.StartingPosition.Right);
    }
}
