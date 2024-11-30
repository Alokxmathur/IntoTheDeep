package org.firstinspires.ftc.teamcode.robot.components.vision.example;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    public static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    public static double fx = 578.272;
    public static double fy = 578.272;
    public static double cx = 402.145;
    public static double cy = 221.506;

    // UNITS ARE METERS
    public static double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    public static final int Left = 1;
    public static final int Middle = 2;
    public static final int Right = 3;

    public static int pipelineStage = 0;
    public static double BLUR_RADIUS = 7;
    public static double HUE_MIN = 0;
    public static double HUE_MAX = 90;
    public static double SATURATION_MIN = 0;
    public static double SATURATION_MAX = 255;
    public static double VALUE_MIN = 0;
    public static double VALUE_MAX = 255;
    public static double MIN_CONTOUR_AREA = 2500;
    public static String BLUR = "Box Blur";
}