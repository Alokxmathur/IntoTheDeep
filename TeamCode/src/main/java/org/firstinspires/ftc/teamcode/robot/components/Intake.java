package org.firstinspires.ftc.teamcode.robot.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.RobotConfig;

import java.util.Locale;

public class Intake {
    DcMotorEx intakeMotor;
    boolean eating = false, expelling = false;

    public Intake(HardwareMap hardwareMap) {

        //initialize our intake motor
        this.intakeMotor = hardwareMap.get(DcMotorEx.class, RobotConfig.INTAKE_MOTOR);
        this.intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    /**
     * Set the inout motor power
     * @param power
     */
    public void setIntakePower(double power) {
        this.intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.intakeMotor.setPower(power);
    }

    public void eat() {
        //set power of intake so it rotates at full speed to bring pixels in
        this.setIntakePower(1);
        this.eating = true;
        this.expelling = false;
    }
    public void abstain() {
        this.intakeMotor.setTargetPosition(
                this.intakeMotor.getCurrentPosition());
        this.intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.intakeMotor.setPower(1);
        this.eating = false;
        this.expelling = false;
    }
    public void release() {
        this.setIntakePower(-1);
        this.eating = false;
        this.expelling = true;
    }
    public void raiseIntake() {

    }
    public void lowerIntake() {

    }
    public boolean isEating() {
        return eating;
    }
    public boolean isExpelling() {
        return expelling;
    }

    public void stop() {
        this.intakeMotor.setPower(0);
    }
    public boolean intakeWithinRange() {
        return (Math.abs(intakeMotor.getTargetPosition() - intakeMotor.getCurrentPosition()) < 5);
    }
    public String getStatus() {
        return String.format(Locale.getDefault(),
                "Intake:%d->%d@%.2f",
                intakeMotor.getCurrentPosition(), intakeMotor.getTargetPosition(), intakeMotor.getPower()
                );
    }
}
