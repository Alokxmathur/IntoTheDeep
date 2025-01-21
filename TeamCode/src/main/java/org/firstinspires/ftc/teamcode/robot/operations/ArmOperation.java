package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.components.Arm;
import org.firstinspires.ftc.teamcode.robot.components.ArmPosition;

import java.util.Date;
import java.util.Locale;

/**
 * This class implements oll of the operations related to managing the arm.
 * The types of operations permitted are:
 *  Open - opens the claw to release sample or specimen
 *  Close - closes the claw to grip sample or specimen
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
        Tucked, Hover, Intake,
        Lower_Basket, Higher_Basket,
        Ascent_Level1, Ascent_Lowered,
        Specimen_Intake,
        High_Chamber, High_Chamber_Deposit,
        Hang_1, Hang_2,
        Release, Hold,
        Extend, Retract, Raise, Lower

    }
    Arm arm;
    Type type;

    boolean shoulderReachedPosition;

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
        else {
            //Match.log(arm.getStatus());
            boolean complete = arm.shoulderIsWithinRange()
                    && (arm.slideIsWithinRange() ||
                        (this.type == Type.High_Chamber_Deposit)
                                && arm.getSlideCurrentPosition() > arm.getSlideTargetPosition())
                && (arm.elbowIsWithinRange() || this.type == Type.High_Chamber_Deposit);
            Match.log(arm.getStatus());
            if (complete && type == Type.Hover) {
                arm.setEncoderValuesAtHover();
            }
            return complete;
        }
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
            ArmPosition armPosition = RobotConfig.ARM_HOVER_POSITION;
            switch (this.type) {
                case Tucked: {
                    armPosition = RobotConfig.ARM_TUCKED_POSITION;
                    break;
                }
                case Hover: {
                    arm.setCurrentExtension(0);
                    arm.setCurrentLowering(0);
                    break;
                }
                case High_Chamber: {
                    armPosition = RobotConfig.ARM_HIGH_CHAMBER_POSITION;
                    break;
                }
                case High_Chamber_Deposit: {
                    armPosition = RobotConfig.ARM_HIGH_CHAMBER_DEPOSIT_POSITION;
                    break;
                }
                case Specimen_Intake: {
                    armPosition = RobotConfig.ARM_SPECIMEN_INTAKE_POSITION;
                    arm.setClawPosition(RobotConfig.CLAW_RELEASE_POSITION);
                    break;
                }
                case Lower_Basket: {
                    armPosition = RobotConfig.ARM_LOWER_BASKET;
                    break;
                }
                case Higher_Basket: {
                    armPosition = RobotConfig.ARM_HIGHER_BASKET;
                    break;
                }
                case Extend: {
                    arm.open();
                    arm.extendAndOrLowerArm(1.0, 0);
                    return;
                }
                case Retract: {
                    arm.open();
                    arm.extendAndOrLowerArm(-1.0, 0);
                    return;
                }
                case Raise: {
                    arm.open();
                    arm.extendAndOrLowerArm(0, -1.0);
                    return;
                }
                case Lower: {
                    arm.open();
                    arm.extendAndOrLowerArm(0, 1.0);
                    return;
                }
                case Intake: {
                    arm.extendAndOrLowerArm(0, 4.0);
                    return;
                }

            }
            arm.setSlidePosition(armPosition.getSlide());
            arm.setShoulderPosition(armPosition.getShoulder());
            arm.setElbowPosition(armPosition.getElbow());
        }
    }

    @Override
    public void abortOperation() {
        arm.stop();
    }
}
