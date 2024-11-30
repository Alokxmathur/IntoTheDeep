package org.firstinspires.ftc.teamcode.robot.operations;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.robot.components.LED;

import java.util.Locale;

public class LedOperation extends Operation {
    LED led;
    RevBlinkinLedDriver.BlinkinPattern type;

    public LedOperation(LED led, RevBlinkinLedDriver.BlinkinPattern type, String title) {
        this.led = led;
        this.type = type;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "Led: --%s",
                this.title);
    }
    @Override
    public boolean isComplete() {
            return true;
    }

    @Override
    public void startOperation() {
        this.led.setPattern(type);
    }

    @Override
    public void abortOperation() {
    }
}
