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
   public static final String SHOULDER = "shoulder";

   public static final String ELBOW = "elbow";

   public static final String CLAW = "claw";

   public static final double CLAW_RELEASE_POSITION = 0.45;
   public static final double CLAW_HOLD_POSITION = 0.05;

   public static final double CAUTIOUS_SPEED = .6;
   public static final double APRIL_TAG_SPEED = .4;
   public static final String OTOS = "otos";

   //Robot center from back is five and half inches away
   public static double ROBOT_CENTER_FROM_BACK = 9.25 * Field.MM_PER_INCH;

   //Robot center from front is four and a half inches
   public static double ROBOT_CENTER_FROM_FRONT = 11.25 * Field.MM_PER_INCH;
   public static final double ROBOT_WIDTH = 14.5 * Field.MM_PER_INCH;

   public static final double ROBOT_LENGTH = ROBOT_CENTER_FROM_BACK + ROBOT_CENTER_FROM_FRONT;

   public static final long SERVO_REQUIRED_TIME = 500; //500 milli-seconds for servo to function

   public static final int ACCEPTABLE_SLIDE_ERROR = 10;
   public static final double MAX_SLIDE_POWER = 1;

   public static final int ACCEPTABLE_SHOULDER_ERROR = 10;
   public static final double MAX_SHOULDER_POWER = 1.0;
   public static final int ACCEPTABLE_ELBOW_ERROR = 3;
   public static final ArmPosition ARM_TUCKED_POSITION =
           new ArmPosition(20, 2500, 0, RobotConfig.CLAW_HOLD_POSITION);
   public static final ArmPosition ARM_HIGH_CHAMBER_POSITION =
           new ArmPosition(44, 3250, 60, RobotConfig.CLAW_HOLD_POSITION);
   public static final ArmPosition ARM_HIGH_CHAMBER_DEPOSIT_POSITION =
           new ArmPosition(1073, 3250, 103, RobotConfig.CLAW_HOLD_POSITION);
   public static final ArmPosition ARM_HOVER_POSITION =
           new ArmPosition(107, 507, 420, RobotConfig.CLAW_RELEASE_POSITION);
   public static final ArmPosition ARM_SPECIMEN_INTAKE_POSITION =
           new ArmPosition(1593, 295,200, RobotConfig.CLAW_RELEASE_POSITION);
   public static final ArmPosition ARM_LOWER_BASKET =
           new ArmPosition(2584, 2438, 0,RobotConfig.CLAW_RELEASE_POSITION);
   public static final ArmPosition ARM_HIGHER_BASKET =
           new ArmPosition(2570, 3180, 450, RobotConfig.CLAW_RELEASE_POSITION);

   public static final int X_PIXEL_COUNT = 1920;
   public static final int Y_PIXEL_COUNT = 1080;
}