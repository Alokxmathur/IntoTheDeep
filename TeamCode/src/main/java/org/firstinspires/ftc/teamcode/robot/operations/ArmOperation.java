package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.components.Arm;
import org.firstinspires.ftc.teamcode.robot.components.ArmPosition;

import java.util.Date;
import java.util.Locale;

import androidx.annotation.NonNull;

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
        Extend, Lower, LowerToSample
    }
    Arm arm;
    Type type;
    double extendBy;
    double lowerBy;


    public void setShoulderSpeed(double shoulderSpeed) {
        this.shoulderSpeed = shoulderSpeed;
    }

    public void setSlideSpeed(double slideSpeed) {
        this.slideSpeed = slideSpeed;
    }

    double shoulderSpeed = RobotConfig.MAX_SHOULDER_POWER;
    double slideSpeed = RobotConfig.MAX_SLIDE_POWER;


    public void setLowerBy(double lowerBy) {
        this.lowerBy = lowerBy;
    }

    public void setExtendBy(double extendBy) {
        this.extendBy = extendBy;
    }

    public ArmOperation(Type type, String title) {
        this.arm = Match.getInstance().getRobot().getArm();
        this.type = type;
        this.title = title;
    }

    @NonNull
    public String toString() {
        return String.format(Locale.getDefault(), "Arm: --%s",
                this.title);
    }

    public boolean isComplete() {
        if (this.type == Type.Hold || this.type == Type.Release) {
            return new Date().getTime() - this.getStartTime().getTime() > RobotConfig.SERVO_REQUIRED_TIME;
        }
        else {
            Match.log(arm.getStatus());
            boolean complete =
                    (arm.shoulderIsWithinRange() || ((this.type == Type.High_Chamber_Deposit || this.type == Type.High_Chamber)
                            && arm.getShoulderCurrentPosition() > arm.getShoulderTargetPosition()))
                    && arm.elbowIsWithinRange()
                    &&
                    (arm.slideIsWithinRange()
                            || (this.type == Type.High_Chamber_Deposit
                                && (arm.getSlideCurrentPosition() > arm.getSlideTargetPosition()
                                    || new Date().getTime() - this.getStartTime().getTime() > 1500)));
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
                    arm.extendAndOrLowerArm(this.extendBy, 0);
                    return;
                }
                case Lower: {
                    arm.open();
                    arm.extendAndOrLowerArm(0, this.lowerBy);
                    return;
                }
                case Intake: {
                    arm.extendAndOrLowerArm(0, 6);
                    return;
                }
                case LowerToSample: {
                    double distanceToSample = 5;//Math.min(arm.getDistanceToObject() / Field.MM_PER_INCH - 1.75, 10);
                    //double distanceToSample = Math.min(arm.getDistanceToObject() / Field.MM_PER_INCH - 1.75, 10);
                    Match.log(String.format(Locale.getDefault(),
                                    "Lowering arm by %.2f inches", distanceToSample/Field.MM_PER_INCH));
                    arm.extendAndOrLowerArm(0, distanceToSample);
                    return;
                }
            }
            arm.setWristPosition(RobotConfig.WRIST_STARTING_POSITION);
            arm.setSlidePosition(armPosition.getSlide(), slideSpeed);
            arm.setShoulderPosition(armPosition.getShoulder(), shoulderSpeed);
            arm.setElbowPosition(armPosition.getElbow());
        }
    }

    @Override
    public void abortOperation() {
        arm.stop();
    }
}
