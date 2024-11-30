package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.components.Arm;
import org.firstinspires.ftc.teamcode.robot.components.Intake;

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
public class IntakeOperation extends Operation {

    public enum Type {
        Eat, Abstain, Release
    }
    Intake intake;
    Type type;

    public IntakeOperation(Type type, String title) {
        this.intake = Match.getInstance().getRobot().getIntake();
        this.type = type;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "Arm: --%s",
                this.title);
    }

    public boolean isComplete() {

       return intake.intakeWithinRange();
    }

    @Override
    public void startOperation() {
        switch (this.type) {
            case Eat: {
                intake.eat();
            }
            case Abstain: {
                intake.abstain();
                break;
            }
            case Release: {
                intake.release();
                break;
            }
        }
    }

    @Override
    public void abortOperation() {
        intake.stop();
    }
}
