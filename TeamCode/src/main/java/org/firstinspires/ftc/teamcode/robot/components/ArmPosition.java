package org.firstinspires.ftc.teamcode.robot.components;

/**
 * This represents the position of our slide with the elbow and the claw
 */
public class ArmPosition {
    public double getClaw() {
        return claw;
    }

    public int getSlide() {
        return slide;
    }

    int slide, shoulder;
    double claw;

    public ArmPosition(int slide, int shoulder, double clawPosition) {
        this.slide = slide;
        this.shoulder = shoulder;
        this.claw = clawPosition;
    }

    public int getShoulder() {
        return shoulder;
    }
}
