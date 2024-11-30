package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="BlueRight", group="Aliyah", preselectTeleOp="Aliyah: Driver Controlled")
public class BlueRight extends Autonomous {
    @Override
    public void init() {
        super.init(telemetry, Alliance.Color.BLUE, Field.StartingPosition.Right);
    }
}
