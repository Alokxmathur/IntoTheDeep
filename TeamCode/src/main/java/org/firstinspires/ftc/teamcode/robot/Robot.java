package org.firstinspires.ftc.teamcode.robot;

import android.util.Log;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.robot.components.Arm;
import org.firstinspires.ftc.teamcode.robot.components.LED;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.robot.components.vision.SilverTitansVisionPortal;
import org.firstinspires.ftc.teamcode.robot.operations.ArmOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DriveToAprilTag;
import org.firstinspires.ftc.teamcode.robot.operations.Operation;
import org.firstinspires.ftc.teamcode.robot.operations.OperationThread;

/**
 * This class represents our robot.
 * <p>
 * It supports the following controls:
 * GamePad1:
 * Left stick - drive, right stick - rotate
 * x - abort pending operations
 * <p>
 * a - lowest arm level
 * b - middle arm level
 * y - top arm level
 * Dpad Up - raise intake platform
 * Dpad Down - lower intake platform
 * Dpad Left - forward rotator
 * Dpad right -backward rotator
 * <p>
 * GamePad2:
 * Left stick - y axis - carousel speed
 * <p>
 * Dpad Up - raise output arm
 * Dpad Down - lower output arm
 * <p>
 * Dpad Left -
 * If right bumper is pressed
 * Open Lid more
 * Else
 * retract output arm
 * Dpad Right - extend output arm
 * If right bumper is pressed
 * Close Lid more
 * Else
 * extend output arm
 * <p>
 * Left trigger -
 * If right bumper is pressed: open to capping position
 * else open bucket
 * Right trigger - close bucket
 * <p>
 * x - if left bumper is pressed, tell output that this is the correct intake level for intake
 * otherwise
 * go to intake position
 * a - if left bumper is pressed, tell output that this is the correct intake level for low level
 * otherwise
 * go to lowest arm level
 * b - if left bumper is pressed, tell output that this is the correct intake level for middle level
 * otherwise
 * go to middle arm level
 * y - if left bumper is pressed, tell output that this is the correct intake level for top level
 * otherwise
 * go to top arm level
 */

public class Robot {

    Telemetry telemetry;
    private HardwareMap hardwareMap;
    Match match;

    OperationThread operationThreadPrimary;
    OperationThread operationThreadSecondary;
    OperationThread operationThreadTertiary;

    //Components
    DriveTrain driveTrain;

    Follower follower;
    LED led;
    Arm arm;
    SilverTitansVisionPortal visionPortal;

    ColorSensor colorSensor;

    //Our sensors etc.

    //our state
    String state = "pre-initialized";

    public Robot() {
        Log.d("SilverTitans", "Robot: got created");
    }

    /**
     * Initialize our robot
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry, Match match) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.match = match;

        //initialize our components
        initVision();
        initDriveTrain();

        this.led = new LED(hardwareMap);
        this.colorSensor = new ColorSensor(hardwareMap);
        if (match.getAlliance() == Alliance.Color.RED) {
            this.led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
        else {
            this.led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }

        this.arm = new Arm(hardwareMap);

        telemetry.addData("Status", "Creating operations thread, please wait");
        telemetry.update();

        Match.log("Started operations threads");
        this.operationThreadPrimary = new OperationThread(this, "Primary", telemetry);
        operationThreadPrimary.start();
        this.operationThreadSecondary = new OperationThread(this, "Secondary", telemetry);
        operationThreadSecondary.start();
        this.operationThreadTertiary = new OperationThread(this, "Tertiary", telemetry);
        operationThreadTertiary.start();
    }

    public void initDriveTrain() {
        //Create our drive train
        telemetry.addData("Status", "Initializing drive train, please wait");
        telemetry.update();
        this.driveTrain = new DriveTrain(hardwareMap);

        follower = new Follower(hardwareMap);
        follower.setMaxPower(.6);
    }

    public void initVision() {
        //initialize Vision
        Match.log("Initializing Vision Portal");
        telemetry.addData("Status", "Initializing Vision Portal, please wait");
        telemetry.update();
        this.visionPortal = new SilverTitansVisionPortal();
        this.visionPortal.init(hardwareMap);
    }

    /**
     * Stop the robot
     */
    public void stop() {
        //Stop all of our motors
        Match.log("Stopping robot");
        if (this.operationThreadPrimary != null) {
            this.operationThreadPrimary.abort();
        }
        if (this.operationThreadSecondary != null) {
            this.operationThreadSecondary.abort();
        }
        if (this.operationThreadTertiary != null) {
            this.operationThreadTertiary.abort();
        }
        if (this.driveTrain != null) {
            this.driveTrain.stop();
        }
        Match.log(("Robot stopped"));
    }

    public void queuePrimaryOperation(Operation operation) {
        this.operationThreadPrimary.queueUpOperation(operation);
    }

    public void queueSecondaryOperation(Operation operation) {
        this.operationThreadSecondary.queueUpOperation(operation);
    }

    public void queueTertiaryOperation(Operation operation) {
        this.operationThreadTertiary.queueUpOperation(operation);
    }

    public boolean allOperationsCompleted() {
        return primaryOperationsCompleted() && secondaryOperationsCompleted() && tertiaryOperationsCompleted();
    }

    public boolean primaryOperationsCompleted() {
        return !this.operationThreadPrimary.hasEntries();
    }

    public boolean secondaryOperationsCompleted() {
        return !this.operationThreadSecondary.hasEntries();
    }

    public boolean tertiaryOperationsCompleted() {
        return !this.operationThreadTertiary.hasEntries();
    }

    public String getState() {
        return this.state;
    }

    public void setState(String state) {
        this.state = state;
    }

    public boolean fullyInitialized() {
        return true;
    }

    public NormalizedRGBA getColors() {
        return this.colorSensor.getColors();
    }

    /*
        gamePad 2 dpad up/down open/close claw incrementally
        gamePad 2 dpad left/right open/close claw totally
    */
    public void handleGameControllers(Gamepad gamePad1, Gamepad gamePad2) {
        if (gamePad1.x) {
            this.operationThreadPrimary.abort();
            this.operationThreadSecondary.abort();
            this.operationThreadTertiary.abort();
        }

        this.handleDriveTrain(gamePad1);
        handleArm(gamePad1, gamePad2);
    }

    /**
     * Handle driving of the robot
     * If the left bumper is pressed, align robot 9 inches to the left of the april tag being seen
     * If the right bumper is pressed, align robot 9 inches to the right of the april tag being seen
     * If both bumpers are pressed, align robot 9 inches centered on the april tag being seen
     * If left or right or both bumpers are pressed robot is attempted to be 12 inches from the
     * april tag
     * If no bumpers are pressed, the left joystick y direction determines forward movement,
     * left joystick x direction determines strafing and the right joy stick x direction
     * determines rotation
     * Right trigger pushing moves robot in turbo mode, left trigger in super turbo mode
     * @param gamePad1 - game pad 1
     */
    public void handleDriveTrain(Gamepad gamePad1) {
        if (this.primaryOperationsCompleted()) {
            double left;
            if (gamePad1.start) {
                this.follower.setPose(new Pose(0, 0 ,0 ));
            }
            if (gamePad1.left_bumper || gamePad1.right_bumper) {
                //if both left and right bumpers are pressed align with the april tag
                if (gamePad1.left_bumper && gamePad1.right_bumper) {
                    left = 0;
                }
                //if only left bumper is pressed, align to the left of the april tag
                else if (gamePad1.left_bumper) {
                    left = 12;
                }
                //if only right bumper is pressed, align to the right tag of the tag
                else {
                    left = -12;
                }
                //align with april tag, staying 10 inches from it
                DriveToAprilTag.driveToAprilTag(left,12*Field.MM_PER_INCH, driveTrain);
            }
            else {
                //regular driving
                double multiplier = gamePad1.right_trigger > 0.1 ? .75 : (gamePad1.left_trigger > 0.1 ? 1 : .5);
                double x = Math.pow(gamePad1.left_stick_x, 5) * multiplier; // Get left joystick's x-axis value.
                double y = -Math.pow(gamePad1.left_stick_y, 5) * multiplier; // Get left joystick's y-axis value.

                double rotation = Math.pow(gamePad1.right_stick_x, 3) * multiplier; // Get right joystick's x-axis value for rotation

                this.driveTrain.drive(Math.atan2(x, y), Math.hypot(x, y), rotation);
            }
        }
    }

    /**
     * Handle the arm

     * @param gamePad2 - game pad 2
     */
    public void handleArm(Gamepad gamePad1, Gamepad gamePad2) {
        if (secondaryOperationsCompleted()) {
            if (gamePad2.a) {
                queueSecondaryOperation(new ArmOperation(ArmOperation.Type.Hover, "Assume Hover"));
            }
            if (gamePad2.b) {
                if (gamePad2.left_trigger > .1) {
                    queueSecondaryOperation(new ArmOperation(ArmOperation.Type.Intake, "Assume Intake"));
                    queueSecondaryOperation(new ArmOperation(ArmOperation.Type.Hold, "Grab"));
                    queueSecondaryOperation(new ArmOperation(ArmOperation.Type.Hover, "Assume Hover"));
                }
                else {
                    queueSecondaryOperation(new ArmOperation(ArmOperation.Type.Intake, "Assume Intake"));
                }
            }
            if (gamePad2.y) {
                queueSecondaryOperation(new ArmOperation(ArmOperation.Type.High_Chamber, "High chamber position"));
            }
            if (gamePad2.x) {
                queueSecondaryOperation(new ArmOperation(ArmOperation.Type.Higher_Basket, "High basket position"));
            }
            if (gamePad1.a) {
                queueSecondaryOperation(new ArmOperation(ArmOperation.Type.Specimen_Intake, "Specimen intake position"));
            }
            if (gamePad1.dpad_up) {
                queueSecondaryOperation(new ArmOperation(ArmOperation.Type.Lower, "Lower arm"));
            }
            if (gamePad1.dpad_down) {
                queueSecondaryOperation(new ArmOperation(ArmOperation.Type.Raise, "Raise arm"));
            }
            if (Math.abs(gamePad2.right_stick_y) > 0.1) {
                //If gamePad2 left or right bumper is pressed, make right_stick work to manage elbow
                if (gamePad2.left_bumper || gamePad2.right_bumper) {
                    //handle elbow movement
                    this.arm.setElbowPower(-gamePad2.right_stick_y);
                    this.arm.retainShoulder();
                } else {
                    this.arm.setShoulderPower(Math.pow(gamePad2.right_stick_y, 3));
                    this.arm.retainSlide();
                }
            }
            else {
                this.arm.retainElbow();
                this.arm.retainShoulder();
            }
            if (Math.abs(gamePad2.left_stick_y) > 0.1) {
                this.arm.setSlidePower(-gamePad2.left_stick_y);
            }
            else {
                this.arm.retainSlide();
            }
                //handle releaser
            if (gamePad2.dpad_up) {
                queueSecondaryOperation(new ArmOperation(ArmOperation.Type.Extend, "Extend"));
            }
            if (gamePad2.dpad_down) {
                queueSecondaryOperation(new ArmOperation(ArmOperation.Type.Retract, "Retract"));
            }
            if (gamePad2.dpad_left) {
                arm.clawReleasePosition();
            }
            if (gamePad2.dpad_right) {
                arm.clawRetainPosition();
            }
        }
    }

    public void reset() {
        if (this.driveTrain != null) {
            this.driveTrain.reset();
        }
        if (this.arm != null) {
            this.arm.ensureMotorDirections();
        }
        initVision();
    }

    public Pose getPose() {
        follower.updatePose();
        return follower.getPose();
    }
    public DriveTrain getDriveTrain() {
        return this.driveTrain;
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        this.led.setPattern(pattern);
    }

    public RevBlinkinLedDriver.BlinkinPattern getLEDStatus() {
        return led.getPattern();
    }

    public String getArmStatus() {
        return this.arm.getStatus();
    }
    public Arm getArm() {
        return this.arm;
    }

    public SilverTitansVisionPortal getVisionPortal() {
        return visionPortal;
    }



    public Follower getFollower() {
        return follower;
    }

    public boolean isInitialized() {
        return arm.armCalibrated();
    }
}