package org.firstinspires.ftc.teamcode.opmodes.drivercontrolled;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.robot.RobotConfig;

@Config
@TeleOp
public class PIDF_Wrist extends OpMode {
    public static double p = 0, i = 0, d = 0, f = 0;

    public static int target = 0;

    private DcMotorEx wristMotor;
    private DcMotorControllerEx motorControllerEx;
    int motorIndex;
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        wristMotor = hardwareMap.get(DcMotorEx.class, RobotConfig.SHOULDER);
        // Get a reference to the motor controller and cast it as an extended functionality controller.
        motorControllerEx = (DcMotorControllerEx) wristMotor.getController();
        motorIndex = ((DcMotorEx) wristMotor).getPortNumber();
    }

    @Override
    public void loop() {
        // change coefficients
        PIDFCoefficients pidfNew = new PIDFCoefficients(p, i, d, f);
        motorControllerEx.setPIDFCoefficients(motorIndex, DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
        int armPosition = wristMotor.getCurrentPosition();

        wristMotor.setTargetPosition(target);

        telemetry.addData("pos ", armPosition);
        telemetry.addData("target ", target);

        telemetry.update();
    }
}
