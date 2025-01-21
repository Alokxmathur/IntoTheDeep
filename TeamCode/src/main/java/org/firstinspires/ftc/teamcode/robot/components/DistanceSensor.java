package org.firstinspires.ftc.teamcode.robot.components;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensor {
    private com.qualcomm.robotcore.hardware.DistanceSensor distanceSensor;

    public DistanceSensor(HardwareMap hardwareMap) {
        // you can use this as a regular DistanceSensor.
        distanceSensor = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, "distanceSensor");
    }
    public double getDistance() {
        return distanceSensor.getDistance(DistanceUnit.MM);
    }
}
