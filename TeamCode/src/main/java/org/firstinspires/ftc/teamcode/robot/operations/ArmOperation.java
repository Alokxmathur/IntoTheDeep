package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.components.Arm;

import java.util.Date;
import java.util.Locale;

/**
 * This class implements oll of the operations related to managing the arm.
 * The types of operations permitted are:
 *  Open - opens the claw to release cone
 *  Close - closes the claw to grip cone
 *  Ground - hold cone at ground junction level
 *  Low - hold cone at low junction level
 *  Mid - hold cone at mid junction level
 *  High - hold cone at high junction level
 *  Pickup - get claw to level to pickup upright cone on the ground
 *  Stack5 - get claw to level to pickup the top (5'th) cone in the stack
 *  Stack4 - get claw to level to pickup the 4'th cone in the stack
 *  Stack3 - get claw to level to pickup the 3'rd cone in the stack
 *  Stack2 - get claw to level to pickup the 2'nd cone in the stack
 *  Stack1 - get claw to level to pickup the bottom cone in the stack - same as ground
 */
public class ArmOperation extends Operation {

    public enum Type {
        Level, Intake,
        Lower_Basket, Higher_Basket,
        Ascent_Level1, Ascent_Lowered,
        Specimen_Intake,
        High_Chamber, High_Chamber_Deposit, High_Chamber_Release,
        Hang_1, Hang_2,
        Release, Hold

    }
    Arm arm;
    Type type;

    boolean shoulderReachedPosition, queuedSlidePosition;

    public ArmOperation(Type type, String title) {
        this.arm = Match.getInstance().getRobot().getArm();
        this.type = type;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "Arm: --%s",
                this.title);
    }

    public boolean isComplete() {
        if (this.type == Type.Hold || this.type == Type.Release) {
            return new Date().getTime() - this.getStartTime().getTime() > RobotConfig.SERVO_REQUIRED_TIME;
        }

        //if the shoulder is within range, we queue up the slide movement
        if (!shoulderReachedPosition && arm.shoulderIsWithinRange()) {
            shoulderReachedPosition = true;
            Match.log("Shoulder reached position, queuing slide movement");
            switch (this.type) {
                case Level: {
                    arm.setSlidePosition(RobotConfig.ARM_LEVEL_POSITION.getSlide());
                    break;
                }
                case Intake: {
                    arm.setSlidePosition(RobotConfig.ARM_INTAKE_POSITION.getSlide());
                    break;
                }
                case High_Chamber: {
                    arm.setSlidePosition(RobotConfig.ARM_HIGH_CHAMBER_POSITION.getSlide());
                    break;
                }
                case High_Chamber_Deposit: {
                    arm.setSlidePosition(RobotConfig.ARM_HIGH_CHAMBER_DEPOSIT_POSITION.getSlide());
                    break;
                }
                case High_Chamber_Release: {
                    arm.setSlidePosition(RobotConfig.ARM_HIGH_CHAMBER_RELEASE_POSITION.getSlide());
                    break;
                }
                case Ascent_Level1:
                case Hang_1: {
                    arm.setSlidePosition(RobotConfig.ARM_ASCENT_LEVEL_1_POSITION.getSlide());
                    break;
                }
                case Ascent_Lowered: {
                    arm.setSlidePosition(RobotConfig.ARM_ASCENT_LOWERED_POSITION.getSlide());
                    break;
                }
                case Specimen_Intake: {
                    arm.setSlidePosition(RobotConfig.ARM_SPECIMEN_INTAKE_POSITION.getSlide());
                    break;
                }
                case Lower_Basket: {
                    arm.setSlidePosition(RobotConfig.ARM_LOWER_BASKET.getSlide());
                    break;
                }
                case Higher_Basket: {
                    arm.setSlidePosition(RobotConfig.ARM_HIGHER_BASKET.getSlide());
                    break;
                }
            }
        }
        //we are done if shoulder had reached its position and the slide has also
        if(shoulderReachedPosition && arm.slideIsWithinRange()) {
            Match.log("Completed with: " + arm.getStatus());
            return true;
        }
        return false;
    }

    @Override
    public void startOperation() {
        if (this.type == Type.Hold) {
            arm.setClawPosition(RobotConfig.CLAW_HOLD_POSITION);
        }
        else if (this.type == Type.Release) {
            arm.setClawPosition(RobotConfig.CLAW_RELEASE_POSITION);
        }
        else {
            //we only make the shoulder go to its position to start with
            switch (this.type) {
                case Level: {
                    arm.setShoulderPosition(RobotConfig.ARM_LEVEL_POSITION.getShoulder());
                    arm.setClawPosition(RobotConfig.ARM_LEVEL_POSITION.getClaw());
                    break;
                }
                case Intake: {
                    arm.setShoulderPosition(RobotConfig.ARM_INTAKE_POSITION.getShoulder());
                    arm.setClawPosition(RobotConfig.ARM_INTAKE_POSITION.getClaw());
                    break;
                }
                case High_Chamber: {
                    arm.setShoulderPosition(RobotConfig.ARM_HIGH_CHAMBER_POSITION.getShoulder());
                    arm.setClawPosition(RobotConfig.ARM_HIGH_CHAMBER_POSITION.getClaw());
                    break;
                }
                case High_Chamber_Deposit: {
                    arm.setShoulderPosition(RobotConfig.ARM_HIGH_CHAMBER_DEPOSIT_POSITION.getShoulder());
                    arm.setClawPosition(RobotConfig.ARM_HIGH_CHAMBER_DEPOSIT_POSITION.getClaw());
                    break;
                }
                case High_Chamber_Release: {
                    arm.setShoulderPosition(RobotConfig.ARM_HIGH_CHAMBER_RELEASE_POSITION.getShoulder());
                    arm.setClawPosition(RobotConfig.ARM_HIGH_CHAMBER_RELEASE_POSITION.getClaw());
                    break;
                }
                case Ascent_Level1:
                case Hang_1: {
                    arm.setShoulderPosition(RobotConfig.ARM_ASCENT_LEVEL_1_POSITION.getShoulder());
                    arm.setClawPosition(RobotConfig.ARM_ASCENT_LEVEL_1_POSITION.getClaw());
                    break;
                }
                case Ascent_Lowered: {
                    arm.setShoulderPosition(RobotConfig.ARM_ASCENT_LOWERED_POSITION.getShoulder());
                    arm.setClawPosition(RobotConfig.ARM_ASCENT_LOWERED_POSITION.getClaw());
                    break;
                }
                case Specimen_Intake: {
                    arm.setShoulderPosition(RobotConfig.ARM_SPECIMEN_INTAKE_POSITION.getShoulder());
                    arm.setClawPosition(RobotConfig.ARM_SPECIMEN_INTAKE_POSITION.getClaw());
                    break;
                }
                case Lower_Basket: {
                    arm.setShoulderPosition(RobotConfig.ARM_LOWER_BASKET.getShoulder());
                    arm.setClawPosition(RobotConfig.ARM_LOWER_BASKET.getClaw());
                    break;
                }
                case Higher_Basket: {
                    arm.setShoulderPosition(RobotConfig.ARM_HIGHER_BASKET.getShoulder());
                    arm.setClawPosition(RobotConfig.ARM_HIGHER_BASKET.getClaw());
                    break;
                }
            }
        }
    }

    @Override
    public void abortOperation() {
        arm.stop();
    }
}
