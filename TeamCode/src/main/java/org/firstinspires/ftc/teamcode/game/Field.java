package org.firstinspires.ftc.teamcode.game;

/**
 * Created by Silver Titans on 9/16/17.
 */
public class Field {
    public static final float MM_PER_INCH = 25.4f;
    public static final float TILE_WIDTH = 24 * MM_PER_INCH;
    public static volatile boolean initialized = true;
    public static final Object mutex = new Object();

    public enum StartingPosition {
        Left, Right, NotSelected
    }
    public enum SpikePosition {
        Left, Middle, Right, NotSeen
    }
    public void init(Alliance.Color alliance, StartingPosition startingPosition) {
    }

    public static boolean isNotInitialized() {
        synchronized (mutex) {
            return !initialized;
        }
    }
}