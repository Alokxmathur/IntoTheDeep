package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.components.Arm;
import org.firstinspires.ftc.teamcode.robot.operations.ArmOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DriveInDirectionOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DriveInDirectionUntilColor;
import org.firstinspires.ftc.teamcode.robot.operations.DriveInDirectionUntilNotColor;
import org.firstinspires.ftc.teamcode.robot.operations.DriveToAprilTag;
import org.firstinspires.ftc.teamcode.robot.operations.FollowPathChain;
import org.firstinspires.ftc.teamcode.robot.operations.GoToPosition;
import org.firstinspires.ftc.teamcode.robot.operations.State;
import org.firstinspires.ftc.teamcode.robot.operations.StrafeLeftForDistanceOperation;
import org.firstinspires.ftc.teamcode.robot.operations.StrafeLeftForDistanceWithHeadingOperation;
import org.firstinspires.ftc.teamcode.robot.operations.StrafeRightForDistanceOperation;
import org.firstinspires.ftc.teamcode.robot.operations.StrafeRightToAprilTagOperation;
import org.firstinspires.ftc.teamcode.robot.operations.TurnAntiClockwiseOperation;
import org.firstinspires.ftc.teamcode.robot.operations.TurnClockwiseOperation;
import org.firstinspires.ftc.teamcode.robot.operations.WaitOperation;

import java.nio.file.Path;

/**
 * This is v2 of the IntoTheDeep autonomous
 * It delivers loaded specimen onto the high chamber, then
 * pushes 2 samples into the observation zone, then
 * picks up specimen made by human player from the wall, then
 * delivers the picked up specimen,
 * goes back to the observation zone to pick up the next specimen placed by human player
 * delivers the second specimen to the high chamber
 * parks in the observation zone
 */
public abstract class AutonomousV2 extends AutonomousHelper {

    public static final double DISTANCE_TO_PUSH_SAMPLES = 44.0 * Field.MM_PER_INCH;
    public static final double DISTANCE_TO_SUBMERSIBLE = 31.3 * Field.MM_PER_INCH;
    double RETRACTION_FROM_WALL = 10 * Field.MM_PER_INCH;
    @Override
    public void start() {
        super.start();
        /**
         * State to deliver specimen to the high chamber
         * This is done by
         * 1. raising and extending the arm for high chamber,
         * 2. moving towards the submersible - this should put the specimen just above the high chamber
         * 3. moving the arm down so specimen snaps onto high chamber
         * 4. moving arm to the release position so it does not get stuck on the chambers
         * 5. retracting from the submersible for latter operations
         */
        State state = new State("Reach submersible");
        state.addPrimaryOperation(new DriveInDirectionOperation(-DISTANCE_TO_SUBMERSIBLE,
                0, RobotConfig.CAUTIOUS_SPEED/2,
                "Reach submersible"));
        state.addSecondaryOperation(new ArmOperation(ArmOperation.Type.High_Chamber, "Arm to chamber position"));
        states.add(state);

        state = new State("Deposit first specimen");
        state.addPrimaryOperation(new ArmOperation(ArmOperation.Type.High_Chamber_Deposit, "Deposit first specimen"));
        state.addPrimaryOperation(new ArmOperation(ArmOperation.Type.Release, "Release specimen"));

        states.add(state);


        //State to reach push one sample and reach top of second

        state = new State("Push first sample");


        state.addPrimaryOperation(new FollowPathChain(Field.redSamplesPathChain, "Push first and reach top of second"));
        state.addSecondaryOperation(new WaitOperation(1500, "Wait to close claw"));
        state.addSecondaryOperation(new ArmOperation(ArmOperation.Type.Hold, "Close claw ready to intake"));
        state.addSecondaryOperation(new ArmOperation(ArmOperation.Type.Specimen_Intake, "Ready to intake specimen"));
        state.setCompletionBasedUpon(State.CompletionBasedUpon.PRIMARY_OPERATIONS);
        states.add(state);

        /*
         * State to push second sample and get ready to grab specimen from wall
         *
         */
        state = new State("Reach specimen on wall");
        state.addPrimaryOperation(new DriveInDirectionOperation(1.4*Field.TILE_WIDTH, 0,
                RobotConfig.CAUTIOUS_SPEED,
                "Avoid color"));
        state.addPrimaryOperation(new DriveInDirectionUntilColor(Field.TILE_WIDTH,
                Math.toRadians(0), .15, "Reach specimen find color"));
        state.addPrimaryOperation(new ArmOperation(ArmOperation.Type.Hold, "Grab specimen"));
        states.add(state);

        state = new State("Grab specimen from wall");
        state.addSecondaryOperation(new ArmOperation(ArmOperation.Type.High_Chamber, "Ready for high chamber"));
        state.addPrimaryOperation(new StrafeRightToAprilTagOperation("Strafe to reach submersible"));
        state.addPrimaryOperation(new DriveToAprilTag(0, DISTANCE_TO_SUBMERSIBLE,
                "Reach submersible"));
        states.add(state);

        state = new State("Deliver second specimen and reach observation zone");
        state.addPrimaryOperation(new ArmOperation(ArmOperation.Type.High_Chamber_Deposit, "Deposit second specimen"));
        state.addPrimaryOperation(new ArmOperation(ArmOperation.Type.Release, "Release specimen"));
        state.addPrimaryOperation(new DriveInDirectionOperation(
                8*Field.MM_PER_INCH,
                0, RobotConfig.CAUTIOUS_SPEED,
                "Clear submersible"));
        states.add(state);

        state = new State("Go for third specimen");
        state.addSecondaryOperation(new ArmOperation(ArmOperation.Type.Specimen_Intake, "Lower arm"));

        state.addPrimaryOperation(new StrafeLeftForDistanceWithHeadingOperation(2*Field.TILE_WIDTH,
                0,  1,
                "Return for third specimen"));
        state.addPrimaryOperation(new TurnClockwiseOperation(0, RobotConfig.CAUTIOUS_SPEED, "Align"));
        state.addPrimaryOperation(new DriveInDirectionUntilColor(12*Field.MM_PER_INCH,
                Math.toRadians(0), 0.15,
                "Reach third specimen"));
        state.addPrimaryOperation(new ArmOperation(ArmOperation.Type.Hold, "Grab specimen"));

        states.add(state);
        state = new State("Deliver 3rd specimen");
        state.addSecondaryOperation(new ArmOperation(ArmOperation.Type.High_Chamber, "Ready for high chamber"));
        state.addPrimaryOperation(new StrafeRightToAprilTagOperation("Strafe to reach submersible"));
        //state.addPrimaryOperation(new TurnAntiClockwiseOperation(0, RobotConfig.CAUTIOUS_SPEED, "align"));
        state.addPrimaryOperation(new DriveInDirectionOperation(-DISTANCE_TO_SUBMERSIBLE/2,
                0,
                RobotConfig.CAUTIOUS_SPEED,
                "Reach submersible"));
        states.add(state);

        state = new State("Deliver third specimen and reach observation zone");
        state.addPrimaryOperation(new ArmOperation(ArmOperation.Type.High_Chamber_Deposit, "Deposit third specimen"));
        state.addPrimaryOperation(new ArmOperation(ArmOperation.Type.Release, "Release specimen"));
        state.addPrimaryOperation(new DriveInDirectionOperation(
                6*Field.MM_PER_INCH,
                0, RobotConfig.CAUTIOUS_SPEED,
                "Clear submersible"));
        state.addPrimaryOperation(new StrafeLeftForDistanceWithHeadingOperation(2.5*Field.TILE_WIDTH,
                0,  RobotConfig.CAUTIOUS_SPEED*2,
                "Park"));
        states.add(state);
    }
}
