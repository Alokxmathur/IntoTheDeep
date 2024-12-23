package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.operations.State;
import org.firstinspires.ftc.teamcode.robot.operations.WaitOperation;

import java.util.ArrayList;
import java.util.Date;

/**
 * This class implements the methods to make autonomous happen
 */
public abstract class AutonomousHelper extends OpMode {
    protected Match match;
    protected Robot robot;
    protected Field field;

    private final Pose redLeftStartingPose = new Pose(
            (6*Field.TILE_WIDTH - RobotConfig.ROBOT_CENTER_FROM_BACK) / Field.MM_PER_INCH,
            (3*Field.TILE_WIDTH-RobotConfig.ROBOT_WIDTH/2) / Field.MM_PER_INCH,
            Math.toRadians(180));
    private final Pose redRightStartingPose = new Pose(
            (6*Field.TILE_WIDTH- RobotConfig.ROBOT_CENTER_FROM_BACK) / Field.MM_PER_INCH,
            (3*Field.TILE_WIDTH+RobotConfig.ROBOT_WIDTH/2)  / Field.MM_PER_INCH,
            Math.toRadians(180));
    private final Pose blueLeftStartingPose = new Pose(
            (RobotConfig.ROBOT_CENTER_FROM_BACK) / Field.MM_PER_INCH,
            (3*Field.TILE_WIDTH+RobotConfig.ROBOT_WIDTH/2) / Field.MM_PER_INCH,
            Math.toRadians(0));
    private final Pose blueRightStartingPose = new Pose(
            (RobotConfig.ROBOT_CENTER_FROM_BACK) / Field.MM_PER_INCH,
            (3*Field.TILE_WIDTH-RobotConfig.ROBOT_WIDTH/2) / Field.MM_PER_INCH,
            Math.toRadians(180));
    protected static WaitOperation delayedStart = null;
    ArrayList<State> states = new ArrayList<>();

    Date initStartTime;
    //start with assuming that there might be an error when initializing the robot
    boolean initErrorHappened = true;
    String initError = "";

    /*
     * Code to run ONCE when the driver hits INIT
     */
    public void init(Telemetry telemetry, Alliance.Color alliance, Field.StartingPosition startingPosition) {
        initStartTime = new Date();
        this.match = Match.getNewInstance();
        match.init();
        Match.log("Match initialized, setting alliance to " + alliance
                + " and starting position to " + startingPosition);
        match.setAlliance(alliance);
        match.setStartingPosition(startingPosition);
        field = match.getField();

        //initialize field for the alliance and starting position
        field.init(alliance, startingPosition);
        //get our robot and initialize it
        this.robot = match.getRobot();
        Match.log("Initializing robot");
        this.robot.init(hardwareMap, telemetry, match);
        this.robot.getFollower().setPose(getStartingPose(alliance, startingPosition));

        //set the starting delay to be 0 milliseconds and add this operation in the first state
        delayedStart = new WaitOperation(0, "Delay start");
        State state = new State("Initial wait");
        state.addPrimaryOperation(delayedStart);
        states.add(state);

        telemetry.update();
        initErrorHappened = false;
    }

    private Pose getStartingPose(Alliance.Color alliance, Field.StartingPosition startingPosition) {
        Pose startingPose;
        if (startingPosition == Field.StartingPosition.Right) {
            if (alliance == Alliance.Color.RED) {
                startingPose = redRightStartingPose;
            }
            else {
                startingPose = blueRightStartingPose;
            }
        }
        else {
            if (alliance == Alliance.Color.RED) {
                startingPose = redLeftStartingPose;
            }
            else {
                startingPose = blueLeftStartingPose;
            }
        }
        return startingPose;
    }

    /*
     * Code to run repeatedly after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        if (match.getAlliance() != Alliance.Color.NotSelected) {
            if (Field.isNotInitialized()) {
                telemetry.addData("State", "Trajectories initializing, please wait. " +
                        (30 - (int) (new Date().getTime() - initStartTime.getTime()) / 1000));
            }
            else {
                //increase or decrease delay in starting operations
                if (gamepad1.dpad_up) {
                    //add one second to the delay in starting operations
                    this.delayedStart.setTime(this.delayedStart.getTime() + 1000);
                }
                else if (gamepad1.dpad_down) {
                    //subtract one second down to a minimum of 0
                    this.delayedStart.setTime(Math.max(this.delayedStart.getTime() - 1000, 0));
                }
                robot.handleGameControllers(gamepad1, gamepad2);
            }
            match.updateTelemetry(telemetry, "Ready");
        }
        telemetry.update();
        Thread.yield();
    }

    @Override
    public void start() {
        match.setStart();
    }

    /**
     * We go through our specified desired states in this method.
     * Loop through the states, checking if a state is reached, if it is not reached, queue
     *         it if not already queued
     */
    @Override
    public void loop() {
        /*
        Check states sequentially. Skip over reached states and queue those that have not
        been reached and not yet queued
         */
        for (State state : states) {
            if (!state.isReached(robot)) {
                if (state.isQueued()) {
                    match.updateTelemetry(telemetry,"Attempting " + state.getTitle());
                } else {
                    //queue state if it has not been queued
                    match.updateTelemetry(telemetry,"Queueing " + state.getTitle());
                    Match.log("Queueing state: " + state.getTitle());
                    state.queue(robot);
                }
                break;
            }
        }
        robot.handleArm(gamepad1, gamepad2);
        robot.handleDriveTrain(gamepad1);
    }

    public static long getStartDelay() {
        return delayedStart.getTime();
    }

    @Override
    public void stop() {
        this.robot.stop();
    }
}