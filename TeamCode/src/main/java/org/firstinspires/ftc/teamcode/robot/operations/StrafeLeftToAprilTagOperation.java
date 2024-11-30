package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.robot.RobotConfig;

public class StrafeLeftToAprilTagOperation extends StrafeRightToAprilTagOperation {

    public StrafeLeftToAprilTagOperation(int desiredAprilTag, String title) {
        super(desiredAprilTag, title);
    }

    @Override
    public void startOperation() {
        this.driveTrain.drive(Math.atan2(-RobotConfig.APRIL_TAG_SPEED, 0), Math.hypot(-RobotConfig.APRIL_TAG_SPEED, 0), 0);
    }
}
