package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.operations.ArmOperation;
import org.firstinspires.ftc.teamcode.robot.operations.BearingOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DriveForDistanceOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DriveInDirectionOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DriveToAprilTag;
import org.firstinspires.ftc.teamcode.robot.operations.IntakeOperation;
import org.firstinspires.ftc.teamcode.robot.operations.LedOperation;
import org.firstinspires.ftc.teamcode.robot.operations.State;
import org.firstinspires.ftc.teamcode.robot.operations.StrafeLeftForDistanceOperation;
import org.firstinspires.ftc.teamcode.robot.operations.StrafeLeftToAprilTagOperation;
import org.firstinspires.ftc.teamcode.robot.operations.StrafeRightForDistanceOperation;
import org.firstinspires.ftc.teamcode.robot.operations.StrafeRightToAprilTagOperation;
import org.firstinspires.ftc.teamcode.robot.operations.WaitOperation;

public abstract class Autonomous extends AutonomousHelper {
    double DISTANCE_TO_SUBMERSIBLE = 30.0 * Field.MM_PER_INCH;
    @Override
    public void start() {
        super.start();
        State state = new State("Deliver Specimen to high chamber");
        state.addPrimaryOperation(
                new ArmOperation(ArmOperation.Type.High_Chamber_1, "Raise arm for high chamber"));
        state.addPrimaryOperation(
                new ArmOperation(ArmOperation.Type.High_Chamber_2, "Extend arm for high chamber"));
        state.addPrimaryOperation(
                new DriveForDistanceOperation(DISTANCE_TO_SUBMERSIBLE, RobotConfig.CAUTIOUS_SPEED, "Reach submersible"));
        state.addPrimaryOperation(
                new ArmOperation(ArmOperation.Type.High_Chamber_Deposit, "Deposit on high chamber"));
        state.addPrimaryOperation(
                new ArmOperation(ArmOperation.Type.High_Chamber_Release, "Release from high chamber"));
        state.addPrimaryOperation(
                new DriveForDistanceOperation(-DISTANCE_TO_SUBMERSIBLE/2, RobotConfig.CAUTIOUS_SPEED, "Retract from submersible"));
        states.add(state);

        state = new State("Reach ascent zone");
        state.addPrimaryOperation(
                new StrafeLeftForDistanceOperation(48*Field.MM_PER_INCH, RobotConfig.CAUTIOUS_SPEED, "Strafe to avoid submersible"));
        state.addPrimaryOperation(new BearingOperation(Math.toRadians(0), match.getRobot().getDriveTrain(), "Realign after strafing"));
        state.addPrimaryOperation(
                new DriveForDistanceOperation(1.25*Field.TILE_WIDTH, RobotConfig.CAUTIOUS_SPEED, "Move toward ascent zone"));
        state.addPrimaryOperation(new BearingOperation(Math.toRadians(-90), match.getRobot().getDriveTrain(), "Rotate to ascent zone"));
        state.addPrimaryOperation(
                new DriveForDistanceOperation(2*Field.TILE_WIDTH, RobotConfig.CAUTIOUS_SPEED, "Move toward ascent zone"));

        state.addSecondaryOperation(
                new ArmOperation(ArmOperation.Type.High_Chamber_1, "Raise arm for high chamber"));
        state.addSecondaryOperation(
                new ArmOperation(ArmOperation.Type.High_Chamber_2, "Extend arm for high chamber"));
        state.addPrimaryOperation(
                new DriveForDistanceOperation(.5*Field.TILE_WIDTH, RobotConfig.CAUTIOUS_SPEED, "Move toward ascent zone"));
        states.add(state);

        state = new State("Touch ascent rung");
        state.addSecondaryOperation(
                new ArmOperation(ArmOperation.Type.High_Chamber_Release, "Lower arm"));
        states.add(state);
    }
}
