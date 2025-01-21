package org.firstinspires.ftc.teamcode.robot.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;

import java.util.Date;
import java.util.Locale;

public class Arm {

    //spool diameter in mms
    public static double spoolDiameter = 24;
    public static int encoderTicksPerRevolutionOfSlide = 560;
    //height of the triangle describing our arm in inches
    public static double heightOfArmAtHover = 6.0;
    public static double hypotenuseOfArmAtHover = 17.1;
    public static double thetaOfArmAtHover = Math.asin(heightOfArmAtHover / hypotenuseOfArmAtHover);

    public static double baseOfArmAtHover =
            Math.sqrt(Math.pow(hypotenuseOfArmAtHover, 2) - Math.pow(heightOfArmAtHover, 2));

    int shoulderEncoderValueAtHover, slideEncoderValueAtHover;

    public static double encodersTicksRequiredPerMMOfExtension =
            encoderTicksPerRevolutionOfSlide / (Math.PI * spoolDiameter);
    public static double encoderTicksRequiredPerInchOfExtension =
            encodersTicksRequiredPerMMOfExtension * Field.MM_PER_INCH;

    public static final int shoulderGearRatio = 100 * 4;

    public static final int encodersTicksRequiredPerRevolutionOfShoulder = 28 * shoulderGearRatio;

    public static final int encoderTicksRequiredPerRadianOfShoulder =
            (int) (encodersTicksRequiredPerRevolutionOfShoulder / Math.toRadians(360));

    boolean shoulderUpperLimitCalibrated,
            shoulderLowerLimitCalibrated,
            shoulderLowered, shoulderUnlowered, shoulderRaised, shoulderUnraised;
    Date calibrationStarted;
    public double currentExtension, currentLowering;

    public double getCurrentExtension() {
        return currentExtension;
    }

    public void setCurrentExtension(double currentExtension) {
        this.currentExtension = currentExtension;
    }
    public double getCurrentLowering() {
        return currentLowering;
    }

    public void setCurrentLowering(double currentLowering) {
        this.currentLowering = currentLowering;
    }



    DcMotorEx slide, shoulder, elbow;

    Servo claw;

    TouchSensor horizontalTouchSensor, verticalTouchSensor;  // Touch sensors

    boolean
            slideRetained,
            shoulderRetained,
            elbowRetained;
    int shoulderUpperLimit;

    public Arm(HardwareMap hardwareMap) {
        //initialize our slide motor
        this.slide = hardwareMap.get(DcMotorEx.class, RobotConfig.SLIDE);
        this.slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //initialize our shoulder motor
        this.shoulder = hardwareMap.get(DcMotorEx.class, RobotConfig.SHOULDER);
        this.shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //initialize our elbow motor
        this.elbow = hardwareMap.get(DcMotorEx.class, RobotConfig.ELBOW);
        this.elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.claw = hardwareMap.get(Servo.class, RobotConfig.CLAW);
        this.claw.setPosition(RobotConfig.CLAW_HOLD_POSITION);

        this.horizontalTouchSensor = hardwareMap.get(TouchSensor.class, "hTouchSensor");
        this.verticalTouchSensor = hardwareMap.get(TouchSensor.class, "vTouchSensor");

        ensureMotorDirections();
        assumeInitialPosition();
    }

    public boolean armCalibrated() {
        if (calibrationStarted == null) {
            calibrationStarted = new Date();
            return false;
        }
        else if (new Date().getTime() - calibrationStarted.getTime() < 2000) {
            return false;
        }
        else if (!shoulderUpperLimitCalibrated) {
            if (!shoulderRaised) {
                //if the vertical limit switch was not pressed, raise shoulder
                if (!verticalTouchSensor.isPressed()) {
                    setShoulderPower(.4);
                } else {
                    Match.log("Shoulder raised");
                    shoulderRaised = true;
                }
            } else if (!shoulderUnraised) {
                //if the vertical limit switch is pressed, lower shoulder
                if (verticalTouchSensor.isPressed()) {
                    setShoulderPower(-.1);
                } else {
                    Match.log("Shoulder un-raised");
                    shoulderUnraised = true;
                }
            } else {
                if (!verticalTouchSensor.isPressed()) {
                    //try to press the upper switch
                    setShoulderPower(.1);
                } else {
                    shoulderUpperLimitCalibrated = true;
                    setShoulderPower(0);
                    //set encoder count to 0 at top of shoulder position
                    this.shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    Match.log("Shoulder upper limit calibrated");

                }
            }
        }
        else if (!shoulderLowerLimitCalibrated) {
            if (!shoulderLowered) {
                //if the horizontal limit switch was not pressed, lower shoulder
                if (!horizontalTouchSensor.isPressed()) {
                    setShoulderPower(-.4);
                } else {
                    Match.log("Shoulder lowered");
                    shoulderLowered = true;
                }
            } else if (!shoulderUnlowered) {
                //if the horizontal limit switch is pressed, raise shoulder
                if (horizontalTouchSensor.isPressed()) {
                    setShoulderPower(.1);
                } else {
                    Match.log("Shoulder un-lowered");
                    shoulderUnlowered = true;
                }
            } else {
                if (!horizontalTouchSensor.isPressed()) {
                    //try to press the lower switch
                    setShoulderPower(-.1);
                    Match.log("Lowering shoulder");
                } else {
                    shoulderLowerLimitCalibrated = true;
                    shoulderUpperLimit = -shoulder.getCurrentPosition();
                    setShoulderPower(0);

                    //set encoder count to 0 at bottom of shoulder position
                    this.shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    retainShoulder();
                    Match.log("Shoulder lower limit calibrated");
                }
            }
        }
        else {
            return true;
        }
        return false;
    }



    public void ensureMotorDirections() {
        this.slide.setDirection(DcMotorSimple.Direction.REVERSE);
        this.shoulder.setDirection(DcMotorSimple.Direction.REVERSE);
        this.elbow.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void assumeInitialPosition() {
        retainElbow();
        retainSlide();
        retainShoulder();
    }

    public void raiseShoulderIncrementally() {
        setShoulderPosition(shoulder.getCurrentPosition() - 5);
    }

    public void lowerShoulderIncrementally() {
        setShoulderPosition(shoulder.getCurrentPosition() + 5);
    }

    public void setClawPosition(double clawPosition) {
        this.claw.setPosition(clawPosition);
    }

    public void stop() {
    }

    private void setPositions(ArmPosition armPosition) {
        setSlidePosition(armPosition.getSlide());
        setShoulderPosition(armPosition.getShoulder());
        setElbowPosition(armPosition.getElbow());
        claw.setPosition(armPosition.getClaw());
    }

    /**
     * Set the slide position
     *
     * @param position
     */
    public void setSlidePosition(int position) {
        this.slide.setTargetPosition(Math.max(position, 0));
        this.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.slide.setPower(RobotConfig.MAX_SLIDE_POWER);
    }

    /**
     * Retain slide in its current position
     */
    public void retainSlide() {
        if (!slideRetained) {
            setSlidePosition(slide.getCurrentPosition());
            slideRetained = true;
        }
    }

    /**
     * Set the slide power
     *
     * @param power
     */
    public void setSlidePower(double power) {
        this.slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.slide.setPower(power);
        slideRetained = false;
    }

    /**
     * Set the shoulder motor position
     *
     * @param position
     */
    public void setShoulderPosition(int position) {
        //we make sure we don't go beyond the upper limit of the shoulder position
        this.shoulder.setTargetPosition(Range.clip(position,0, shoulderUpperLimit));
        this.shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.shoulder.setPower(RobotConfig.MAX_SHOULDER_POWER);
    }

    /**
     * Retain shoulder in its current position
     */
    public void retainShoulder() {
        if (!shoulderRetained) {
            setShoulderPosition(shoulder.getCurrentPosition());
            shoulderRetained = true;
        }
    }

    /**
     * Set the shoulder power
     *
     * @param power
     */
    public void setShoulderPower(double power) {
        this.shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.shoulder.setPower(power * RobotConfig.MAX_SHOULDER_POWER);
        shoulderRetained = false;
    }

    /**
     * Set the elbow motor position
     *
     * @param position
     */
    public void setElbowPosition(int position) {
        this.elbow.setTargetPosition(position);
        this.elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.elbow.setPower(RobotConfig.MAX_SHOULDER_POWER);
    }

    /**
     * Retain elbow in its current position
     */
    public void retainElbow() {
        if (!elbowRetained) {
            setElbowPosition(elbow.getCurrentPosition());
            elbowRetained = true;
        }
    }

    /**
     * Set the elbow power
     *
     * @param power
     */
    public void setElbowPower(double power) {
        this.elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.elbow.setPower(power * RobotConfig.MAX_SHOULDER_POWER);
        elbowRetained = false;
    }


    /**
     * Returns true if slide and shoulder are within range
     *
     * @return
     */
    public boolean isWithinRange() {

        return slideIsWithinRange() && shoulderIsWithinRange() && elbowIsWithinRange();
    }

    public boolean slideIsWithinRange() {
        return Math.abs(slide.getTargetPosition() - slide.getCurrentPosition()) <= RobotConfig.ACCEPTABLE_SLIDE_ERROR;
    }

    public boolean shoulderIsWithinRange() {
        return Math.abs(shoulder.getTargetPosition() - shoulder.getCurrentPosition()) <= RobotConfig.ACCEPTABLE_SHOULDER_ERROR;
    }

    public boolean elbowIsWithinRange() {
        return Math.abs(elbow.getTargetPosition() - elbow.getCurrentPosition()) <= RobotConfig.ACCEPTABLE_ELBOW_ERROR;
    }


    /**
     * Returns the status of the arm
     * Reports the current position, target position and power of the shoulder,
     * current position, target position and power of the slide,
     * the in-out motor's speed
     * the position of the wrist and the position of the sorter
     * the state of the shoulder limit switch
     *
     * @return
     */
    public String getStatus() {
        return String.format(Locale.getDefault(),
                "Ext:%.2f,Lowering:%.2f,Slide:%d->%d@%.2f, Shld:%d->%d@%.2fMax(%d), Elb:%d->%d@%.2f, Clw:%.2f, TH:%s, TV:%s",
                currentExtension, currentLowering,
                slide.getCurrentPosition(), slide.getTargetPosition(), slide.getPower(),
                shoulder.getCurrentPosition(), shoulder.getTargetPosition(), shoulder.getPower(), this.shoulderUpperLimit,
                elbow.getCurrentPosition(), elbow.getTargetPosition(), elbow.getPower(),
                claw.getPosition(), horizontalTouchSensor.isPressed() ? "Pressed" : "Not pressed",
                horizontalTouchSensor.isPressed() ? "Pressed" : "Not pressed");
    }

    public void clawReleasePosition() {
        this.claw.setPosition(RobotConfig.CLAW_RELEASE_POSITION);
    }

    public void clawRetainPosition() {
        this.claw.setPosition(RobotConfig.CLAW_HOLD_POSITION);
    }

    public void incrementReleaserPosition() {
        this.claw.setPosition(this.claw.getPosition() + RobotConfig.SERVO_INCREMENT);
    }

    public void decrementReleaserPosition() {
        this.claw.setPosition(this.claw.getPosition() - RobotConfig.SERVO_INCREMENT);
    }
    public double thetaAtHover() {
        return getTheta(0, 0);
    }
    public double hypotenuseAtHover() {
        return getHypotenuse(0, 0);
    }
    public double getTheta(double extension, double lowering) {
        return Math.atan((heightOfArmAtHover + lowering) / (baseOfArmAtHover + extension));
    }
    public double getHypotenuse(double extension, double lowering) {
        return Math.sqrt(Math.pow((heightOfArmAtHover + lowering) ,2) + Math.pow(baseOfArmAtHover + extension, 2));
    }

    //extend arm by provided inches
    public void extendAndOrLowerArm(double inchesToExtendBy, double inchesToLowerBy) {
        //record the extension and lowering we are about to do and calculate new theta and hypotenuse
        this.currentExtension += inchesToExtendBy;
        this.currentLowering += inchesToLowerBy;
        //get the required theta and hypotenuse for the updated extension and height
        double newTheta = getTheta(currentExtension, currentLowering);
        double newHypotenuse = getHypotenuse(currentExtension, currentLowering);

        double hypotenuseChangeRequired = newHypotenuse - hypotenuseAtHover();
        double thetaChangeRequired =  thetaAtHover() - newTheta;

        Match.log(String.format(Locale.getDefault(),
                "Hypotenuse from hover of %.2f->%.2f, theta: of %.2f->%.2f",
                hypotenuseAtHover(), newHypotenuse,
                thetaAtHover(), newTheta));
        int slideEncoderValueIncrement = (int)
                (hypotenuseChangeRequired * encoderTicksRequiredPerInchOfExtension);
        setSlidePosition(slideEncoderValueIncrement + slideEncoderValueAtHover);

        int shoulderEncoderValueIncrement = (int)
                (thetaChangeRequired * encoderTicksRequiredPerRadianOfShoulder);
        setShoulderPosition(shoulderEncoderValueIncrement + shoulderEncoderValueAtHover);
        Match.log("Extending and/or lowering arm: " + getStatus());
    }

    public void setEncoderValuesAtHover() {
        this.shoulderEncoderValueAtHover = this.shoulder.getCurrentPosition();
        this.slideEncoderValueAtHover = this.slide.getCurrentPosition();
        Match.log("Setting encoder values at hover, slide = " + slideEncoderValueAtHover
            + ", shoulder=" + shoulderEncoderValueAtHover);
    }

    public void open() {
        this.claw.setPosition(RobotConfig.CLAW_RELEASE_POSITION);
    }

    public int getSlideCurrentPosition() {
        return this.slide.getCurrentPosition();
    }
    public int getSlideTargetPosition() {
        return this.slide.getTargetPosition();
    }
}