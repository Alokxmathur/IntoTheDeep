package org.firstinspires.ftc.teamcode.robot.components.drivetrain;


import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.operations.DriveForDistanceOperation;
import org.firstinspires.ftc.teamcode.robot.operations.SlopingTurnForTimeOperation;
import org.firstinspires.ftc.teamcode.robot.operations.StrafeLeftForDistanceOperation;
import org.firstinspires.ftc.teamcode.robot.operations.StrafeLeftForDistanceWithHeadingOperation;
import org.firstinspires.ftc.teamcode.robot.operations.StrafeRightForDistanceOperation;
import org.firstinspires.ftc.teamcode.robot.operations.StrafeRightToAprilTagOperation;

import java.util.Locale;

/**
 * Created by Silver Titans on 10/26/17.
 */

public class DriveTrain extends SparkFunOTOSDrive {
    public static final int WITHIN_RANGE = 30;
    public static final double P_DRIVE_COEFFICIENT = 0.0125 * Math.PI;     // Larger is more responsive, but also less stable


    public DriveTrain(HardwareMap hardwareMap) {
        super(hardwareMap, new Pose2d(0, 0, 0));
    }

    public Pose2d getPose() {
        updatePoseEstimate();
        return super.pose;
    }

        /** Set power of left front motor
         *
         * @param power - the power to set
         *
         */
    public void setLeftFrontPower(double power) {
        //super.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        super.leftFront.setPower(power);
    }

    /**
     * Set power of right front motor
     * @param power - the power to set
     */
    public void setRightFrontPower(double power) {
        //super.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        super.rightFront.setPower(power);
    }

    /** Set power of left Back motor
     *
     * @param power - the power to set
     *
     */
    public void setLeftBackPower(double power) {
        //super.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        super.leftBack.setPower(power);
    }

    /**
     * Set power of right Back motor
     * @param power - the power to set
     */
    public void setRightBackPower(double power) {
        //super.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        super.rightBack.setPower(power);
    }

    /**
     * Handle operation to drive for the specified distance in the direction the robot is facing
     * @param operation
     *
     * We do this by computing how much each wheel must be rotated to travel the specified distance
     * and then commanding the motors to reach the new desired encoder values
     *
     */
    public void handleOperation(DriveForDistanceOperation operation) {
        stop();

        int encoderChange = SilverTitansDriveConstants.mmToEncoderTicks(operation.getDistance());
        this.leftFront.setTargetPosition(encoderChange);
        this.rightFront.setTargetPosition(encoderChange);
        this.leftBack.setTargetPosition(encoderChange);
        this.rightBack.setTargetPosition(encoderChange);

        this.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.leftFront.setPower(operation.getSpeed());
        this.rightFront.setPower(operation.getSpeed());
        this.leftBack.setPower(operation.getSpeed());
        this.rightBack.setPower(operation.getSpeed());
    }

    /**
     * Handle a sloping turn operation by setting the speeds of the left and right motors to
     * the specified values
     * @param operation
     */
    public void handleOperation(SlopingTurnForTimeOperation operation) {
        stop();

        this.leftFront.setPower(operation.getLeftSpeed());
        this.rightFront.setPower(operation.getRightSpeed());
        this.leftBack.setPower(operation.getLeftSpeed());
        this.rightBack.setPower(operation.getRightSpeed());
    }
    /**
     * Handle operation to strafe left for the specified distance perpendicular to the direction the robot is facing
     * @param operation
     *
     * We do this by computing how much each wheel must be rotated to travel the specified distance
     * and then commanding the motors to reach the new desired encoder values
     *
     * We make the left front and right Back motors move forward while making the right front and left Back
     * motors propel backwards
     *
     */
    public void handleOperation(StrafeLeftForDistanceOperation operation) {
        stop();
        int encoderChange = SilverTitansDriveConstants.mmToEncoderTicks(operation.getDistance() * 1.05);
        this.leftFront.setTargetPosition(leftFront.getCurrentPosition() - encoderChange);
        this.rightFront.setTargetPosition(rightFront.getCurrentPosition() + encoderChange);
        this.leftBack.setTargetPosition(leftBack.getCurrentPosition() + encoderChange);
        this.rightBack.setTargetPosition(rightBack.getCurrentPosition() - encoderChange);

        this.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.leftFront.setPower(operation.getSpeed());
        this.rightFront.setPower(operation.getSpeed());
        this.leftBack.setPower(operation.getSpeed());
        this.rightBack.setPower(operation.getSpeed());
    }

    /**
     * Handle operation to strafe for the specified distance perpendicular to the direction the robot is facing
     * @param operation
     *
     * We do this by computing how much each wheel must be rotated to travel the specified distance
     * and then commanding the motors to reach the new desired encoder values
     *
     * We make the left front and right Back motors move forward while making the right front and left Back
     * motors propel backwards
     *
     */
    public void handleOperation(StrafeLeftForDistanceWithHeadingOperation operation) {
        stop();
        int encoderChange = SilverTitansDriveConstants.mmToEncoderTicks(operation.getDistance() * 1.05);
        this.leftFront.setTargetPosition(leftFront.getCurrentPosition() - encoderChange);
        this.rightFront.setTargetPosition(rightFront.getCurrentPosition() + encoderChange);
        this.leftBack.setTargetPosition(leftBack.getCurrentPosition() + encoderChange);
        this.rightBack.setTargetPosition(rightBack.getCurrentPosition() - encoderChange);

        this.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void handleOperation(StrafeRightForDistanceOperation operation) {
        stop();
        int encoderChange = SilverTitansDriveConstants.mmToEncoderTicks(operation.getDistance() * 1.05);
        this.leftFront.setTargetPosition(leftFront.getCurrentPosition() + encoderChange);
        this.rightFront.setTargetPosition(rightFront.getCurrentPosition() - encoderChange);
        this.leftBack.setTargetPosition(leftBack.getCurrentPosition() - encoderChange);
        this.rightBack.setTargetPosition(rightBack.getCurrentPosition() + encoderChange);

        this.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.leftFront.setPower(operation.getSpeed());
        this.rightFront.setPower(operation.getSpeed());
        this.leftBack.setPower(operation.getSpeed());
        this.rightBack.setPower(operation.getSpeed());
    }


    private boolean withinRange(DcMotor... motors) {
        for (DcMotor motor: motors) {
            if (Math.abs(motor.getTargetPosition() - motor.getCurrentPosition()) <= WITHIN_RANGE) {
                return true;
            }
        }
        return false;
    }

    private boolean allWithinRange(DcMotor... motors) {
        for (DcMotor motor: motors) {
            if (Math.abs(motor.getTargetPosition() - motor.getCurrentPosition()) > WITHIN_RANGE) {
                return false;
            }
        }
        return true;
    }
    private boolean withinRange()  {
        return withinRange(leftFront, rightFront, leftBack, rightBack);
    }

    public boolean allWithinRange()  {
        if (allWithinRange(leftFront, rightFront, leftBack, rightBack)) {
            stop();
            return true;
        }
        return false;
    }

    /**
     * Check if the drive train is within the specified encoder count
     * @return - true if the drive train is within encoder tolerance
     */
    public boolean driveTrainWithinRange() {
        if (withinRange())
        {
            stop();
            return true;
        }
        else {
            return false;
        }
    }

    public void stop() {
        //Stop our motors
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        reset();
    }

    public void reset() {
        resetEncoders();
        this.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ensureMotorDirections();
    }

    private void resetEncoders() {
        this.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Match.log("Reset drivetrain encoders");
    }

    public String getStatus() {
        return String.format(Locale.getDefault(),
                "LF:%.2f(%d>%d),RF:%.2f(%d>%d),LR:%.2f(%d>%d),RR:%.2f(%d>%d)",
            this.leftFront.getPower(), this.leftFront.getCurrentPosition(), this.leftFront.getTargetPosition(),
            this.rightFront.getPower(), this.rightFront.getCurrentPosition(), this.rightFront.getTargetPosition(),
            this.leftBack.getPower(), this.leftBack.getCurrentPosition(), this.leftBack.getTargetPosition(),
            this.rightBack.getPower(), this.rightBack.getCurrentPosition(), this.rightBack.getTargetPosition());
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param coefficient  Proportional Gain Coefficient
     * @return - desired steering force
     */
    public static double getSteer(double error, double coefficient) {
        return Range.clip(error * coefficient, -1, 1);
    }

    /**
     * Drive in the specified direction at the specified speed while rotating at the specified rotation
     * Direction is relative to the robot
     * @param direction - direction to drive
     * @param speed - speed at which to drive
     * @param rotation - how much to rotate while driving
     */
    public void drive(double direction, double speed, double rotation) {
        double sin = Math.sin(direction + Math.PI / 4.0);
        double cos = Math.cos(direction + Math.PI / 4.0);
        double max = Math.max(Math.abs(sin), Math.abs(cos));
        sin /= max;
        cos /= max;

        double v1 = speed * sin + rotation;
        double v2 = speed * cos - rotation;
        double v3 = speed * cos + rotation;
        double v4 = speed * sin - rotation;

        // Ensure that none of the values go over 1.0. If none of the provided values are
        // over 1.0, just scale by 1.0 and keep all values.
        double scale = max(1.0, v1, v2, v3, v4);
        if (scale > 1) {
            v1 /= scale;
            v2 /= scale;
            v3 /= scale;
            v4 /= scale;
        }

        setLeftFrontPower(v1);
        setRightFrontPower(v2);
        setLeftBackPower(v3);
        setRightBackPower(v4);

        //Match.log(String.format(Locale.getDefault(), "Powers: LF:%.2f,RF:%.2f,LR:%.2f,RR:%.2f", v1, v2, v3, v4));
    }


    /// Maximum absolute value of some number of arguments
    public static double max(double... xs) {
        double ret = 0.0;
        for (double x : xs) {
            ret = Math.max(ret, Math.abs(x));
        }
        return ret;
    }

    public void handleOperation(StrafeRightToAprilTagOperation strafeRightToAprilTagOperation) {
    }
}
