package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.operations.ArmOperation;
import org.firstinspires.ftc.teamcode.robot.operations.BearingOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DriveForDistanceOperation;
import org.firstinspires.ftc.teamcode.robot.operations.State;
import org.firstinspires.ftc.teamcode.robot.operations.StrafeLeftForDistanceOperation;

public abstract class Autonomous extends AutonomousHelper {
    double DISTANCE_TO_SUBMERSIBLE = 30.0 * Field.MM_PER_INCH;
    @Override
    public void start() {
        super.start();
        State state = new State("Deliver Specimen to high chamber");
        state.addPrimaryOperation(
                new ArmOperation(ArmOperation.Type.High_Chamber, "Raise arm for high chamber"));
        state.addPrimaryOperation(
                new DriveForDistanceOperation(DISTANCE_TO_SUBMERSIBLE, RobotConfig.CAUTIOUS_SPEED, "Reach submersible"));
        state.addPrimaryOperation(
                new ArmOperation(ArmOperation.Type.High_Chamber_Deposit, "Release specimen"));
        state.addPrimaryOperation(
                new DriveForDistanceOperation(-DISTANCE_TO_SUBMERSIBLE/2, RobotConfig.CAUTIOUS_SPEED, "Retract from submersible"));
        states.add(state);

        state = new State("Reach ascent zone");
        state.addPrimaryOperation(
                new StrafeLeftForDistanceOperation(48*Field.MM_PER_INCH, RobotConfig.CAUTIOUS_SPEED, "Strafe to avoid submersible"));
        state.addPrimaryOperation(new BearingOperation(Math.toRadians(0), "Realign after strafing"));
        state.addPrimaryOperation(
                new DriveForDistanceOperation(1.25*Field.TILE_WIDTH, RobotConfig.CAUTIOUS_SPEED, "Move toward ascent zone"));
        state.addPrimaryOperation(new BearingOperation(Math.toRadians(-90), "Rotate to ascent zone"));
        state.addPrimaryOperation(
                new DriveForDistanceOperation(2*Field.TILE_WIDTH, RobotConfig.CAUTIOUS_SPEED, "Move toward ascent zone"));

        state.addSecondaryOperation(
                new ArmOperation(ArmOperation.Type.High_Chamber, "Raise arm for high chamber"));
        state.addPrimaryOperation(
                new DriveForDistanceOperation(.5*Field.TILE_WIDTH, RobotConfig.CAUTIOUS_SPEED, "Move toward ascent zone"));
        states.add(state);

        state = new State("Touch ascent rung");
        state.addSecondaryOperation(
                new ArmOperation(ArmOperation.Type.High_Chamber_Deposit, "Lower arm"));
        states.add(state);
    }
}
