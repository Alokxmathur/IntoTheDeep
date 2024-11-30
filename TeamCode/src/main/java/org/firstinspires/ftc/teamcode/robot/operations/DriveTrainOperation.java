package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.DriveTrain;

public abstract class DriveTrainOperation extends Operation {
    DriveTrain driveTrain;
    public DriveTrainOperation() {
        super();
        this.driveTrain = Match.getInstance().getRobot().getDriveTrain();
    }

    @Override
    public void abortOperation() {
        try {
            driveTrain.stop();
        }
        catch (Throwable e) {}
        Match.log("Aborted " + this.title);
    }
}
