package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.robot.components.ArmPosition;

public class RobotConfig {
   public static final double SERVO_INCREMENT = 0.005;
   //drive train motors
   public static final String LEFT_FRONT_DRIVE = "leftFrontDrive";
   public static final String LEFT_REAR_DRIVE = "leftRearDrive";
   public static final String RIGHT_REAR_DRIVE = "rightRearDrive";
   public static final String RIGHT_FRONT_DRIVE = "rightFrontDrive";
   public static final String WEBCAM_ID = "Webcam 1";

   public static final String SLIDE = "slide";
   public static final String INTAKE_MOTOR = "intakeMotor";
   public static final String SHOULDER = "shoulder";

   public static final String CLAW = "claw";

   public static final String COLOR_SENSOR = "distanceSensor";

   public static final double CLAW_RELEASE_POSITION = 0.42;
   public static final double CLAW_HOLD_POSITION = 0.05;

   public static final double CAUTIOUS_SPEED = 0.6;
   public static final double APRIL_TAG_SPEED = 1.0;
   public static final String OTOS = "otos";

   //Robot center from back is five and half inches away
   public static double ROBOT_CENTER_FROM_BACK = 17.375 * Field.MM_PER_INCH/2;

   //Robot center from front is four and a half inches
   public static double ROBOT_CENTER_FROM_FRONT = 17.375 * Field.MM_PER_INCH/2;
   public static final double ROBOT_WIDTH = 14.5 * Field.MM_PER_INCH;

   public static final double ROBOT_LENGTH = ROBOT_CENTER_FROM_BACK + ROBOT_CENTER_FROM_FRONT;

   public static final long SERVO_REQUIRED_TIME = 500; //500 milli-seconds for servo to function

   public static final int ACCEPTABLE_SLIDE_ERROR = 10;
   public static final double MAX_SLIDE_POWER = 1;

   public static final int ACCEPTABLE_SHOULDER_ERROR = 10;
   public static final double MAX_SHOULDER_POWER = 1.0;

   public static final ArmPosition ARM_LEVEL_POSITION =
           new ArmPosition(0, 2400, RobotConfig.CLAW_HOLD_POSITION);
   public static final ArmPosition ARM_HIGH_CHAMBER_POSITION =
           new ArmPosition(2616, 2400, RobotConfig.CLAW_HOLD_POSITION);
   public static final ArmPosition ARM_HIGH_CHAMBER_DEPOSIT_POSITION =
           new ArmPosition(ARM_HIGH_CHAMBER_POSITION.getSlide(), 1800, RobotConfig.CLAW_HOLD_POSITION);
   public static final ArmPosition ARM_HIGH_CHAMBER_RELEASE_POSITION =
           new ArmPosition(ARM_HIGH_CHAMBER_POSITION.getSlide(), 2185, RobotConfig.CLAW_RELEASE_POSITION);
   public static final ArmPosition ARM_ASCENT_LEVEL_1_POSITION =
           new ArmPosition(1100, 1980, RobotConfig.CLAW_HOLD_POSITION);
   public static final ArmPosition ARM_ASCENT_LOWERED_POSITION =
           new ArmPosition(1100, 1600, RobotConfig.CLAW_HOLD_POSITION);
   public static final ArmPosition ARM_INTAKE_POSITION =
           new ArmPosition(1500, 1030, RobotConfig.CLAW_RELEASE_POSITION);
   public static final ArmPosition ARM_SPECIMEN_INTAKE_POSITION =
           new ArmPosition(2050, 1265, RobotConfig.CLAW_RELEASE_POSITION);

   public static final ArmPosition ARM_LOWER_BASKET =
           new ArmPosition(2584, 2438, RobotConfig.CLAW_RELEASE_POSITION);
   public static final ArmPosition ARM_HIGHER_BASKET =
           new ArmPosition(3469, 3334, RobotConfig.CLAW_RELEASE_POSITION);


   public static final int X_PIXEL_COUNT = 1920;
   public static final int Y_PIXEL_COUNT = 1080;
}