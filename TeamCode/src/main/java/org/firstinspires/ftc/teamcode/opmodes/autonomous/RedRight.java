package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="RedRight", group="Aliyah", preselectTeleOp="Aliyah: Driver Controlled")
public class RedRight extends Autonomous {
    @Override
    public void init() {
        super.init(telemetry, Alliance.Color.RED, Field.StartingPosition.Right);
    }
}
