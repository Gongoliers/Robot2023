// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.PIDConstants;
import com.thegongoliers.commands.DoNothingCommand;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.ArmState;
import frc.lib.swerve.COTSFalconSwerveConstants;
import frc.lib.swerve.SwerveModuleConfig;
import java.util.HashMap;
import java.util.Map;

public final class Constants {

  public static final int PNEUMATICS_HUB_ID = 30;

  public static final class Driver {
    /** Port in Driver Station for the driver controller. */
    public static final int CONTROLLER_PORT = 0;
    /** Minimum axis displacement to register movement. */
    public static final double DEADBAND = 0.2;
    /** Minimum trigger displacement to register a press. */
    public static final double TRIGGER_THRESHOLD = 0.5;
    /** Minimum number of seconds for an active trigger to be considered active. */
    public static final double DEBOUNCE_SECONDS = 0.1;
    /** Axis for forward-backward movement. */
    public static final XboxController.Axis LEFT_VERTICAL_AXIS = XboxController.Axis.kLeftY;
    /** Axis for left-right movement. */
    public static final XboxController.Axis LEFT_HORIZONTAL_AXIS = XboxController.Axis.kLeftX;
    /** Axis for controlling heading. */
    public static final XboxController.Axis RIGHT_VERTICAL_AXIS = XboxController.Axis.kRightY;
    /** Axis for controlling heading. */
    public static final XboxController.Axis RIGHT_HORIZONTAL_AXIS = XboxController.Axis.kRightX;
    /** Button for zeroing the gyro. */
    public static final XboxController.Button ZERO_GYRO_BUTTON = XboxController.Button.kY;
    /** Button for setting the swerve module into "cross mode" so it cannot be pushed */
    public static final XboxController.Button CROSS_BUTTON = XboxController.Button.kX;
  }

  public static final class Manipulator {
    /** Port in Driver Station for the driver controller. */
    public static final int CONTROLLER_PORT = 1;
    /** Minimum trigger displacement to register a press. */
    public static final double TRIGGER_THRESHOLD = 0.8;
    /** Button for extending to the floor state. */
    public static final XboxController.Button FLOOR_BUTTON = XboxController.Button.kX;
    /** Button for extending to the middle row state. */
    public static final XboxController.Button MIDDLE_BUTTON = XboxController.Button.kA;
    /** Button for extending to the top row state. */
    public static final XboxController.Button TOP_BUTTON = XboxController.Button.kY;
    /** Button for extending to the substation pickup state. */
    public static final XboxController.Button SUBSTATION_BUTTON = XboxController.Button.kB;
    /** Button for intaking. */
    public static final XboxController.Axis CLOSE_AXIS = XboxController.Axis.kLeftTrigger;
    /** Button for outtaking. */
    public static final XboxController.Axis OPEN_AXIS = XboxController.Axis.kRightTrigger;
    /** Axis for rotating the arm up and down. */
    public static final XboxController.Axis RAISE_LOWER_AXIS = XboxController.Axis.kLeftY;
    /** Axis for extending and retracting the arm. */
    public static final XboxController.Axis EXTEND_RETRACT_AXIS = XboxController.Axis.kRightY;
  }

  public static final class Swerve {
    /**
     * Name of the CAN bus for all swerve devices. View the name of the CANivore on Phoenix Tuner
     * and match this constant to that name.
     */
    public static final String CANBUS_NAME = "Drivetrain";
    /**
     * CAN ID of the Pigeon 2. Locate the correct Pigeon 2 in Phoenix Tuner using the Blink button,
     * then copy the ID of the Pigeon 2 to this constant.
     */
    public static final int PIGEON_ID = 7;
    /**
     * Toggle for if the Pigeon 2 is CCW+ CW-. Calibrate by rotating the Pigeon 2 counter-clockwise,
     * and checking that the angle value increases.
     */
    public static final boolean SHOULD_INVERT_GYRO = false;

    /**
     * Gear ratio of the drive motor. Use the gear ratio listed in the manufacturer's documentation.
     */
    public static final double DRIVE_MOTOR_GEAR_RATIO_SPEC =
        COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L3;
    /** Swerve module type. Contains the constants that make motors operate correctly. */
    public static final COTSFalconSwerveConstants COTS_MODULE_TYPE =
        COTSFalconSwerveConstants.SDSMK4i(DRIVE_MOTOR_GEAR_RATIO_SPEC);

    /**
     * Center-to-center distance of left and right modules, in meters. Measure the shaft-shaft
     * distance in CAD.
     */
    public static final double TRACK_WIDTH = Units.inchesToMeters(22.75);
    /**
     * Center-to-center distance of front and rear modules, in meters. Measure the shaft-shaft
     * distance in CAD.
     */
    public static final double WHEEL_BASE = Units.inchesToMeters(22.75);
    /**
     * Cirumference of the wheel (including tread) in meters. Measure the wheel's physical
     * dimensions.
     */
    public static final double WHEEL_CIRCUMFERENCE = COTS_MODULE_TYPE.wheelCircumference;

    /** Inverse kinematics helper class. Calculated from the center-center distances. */
    public static final SwerveDriveKinematics KINEMATICS =
        new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

    /**
     * Gear ratio for the drive motor. Use the gear ratio listed in the manufacturer's
     * documentation.
     */
    public static final double DRIVE_MOTOR_GEAR_RATIO = COTS_MODULE_TYPE.driveGearRatio;
    /**
     * Gear ratio for the angle motor. Use the gear ratio listed in the manufacturer's
     * documentation.
     */
    public static final double ANGLE_MOTOR_GEAR_RATIO = COTS_MODULE_TYPE.angleGearRatio;

    /** Toggle for angle motor CCW+. Dependent on module type. */
    public static final boolean SHOULD_INVERT_ANGLE_MOTOR = COTS_MODULE_TYPE.angleMotorInvert;
    /** Toggle for drive motor CCW+. Dependent on module type. */
    public static final boolean SHOULD_INVERT_DRIVE_MOTOR = COTS_MODULE_TYPE.driveMotorInvert;
    /** Toggle for CANCoder CCW+. Dependent on module type. */
    public static final boolean SHOULD_INVERT_CANCODER = COTS_MODULE_TYPE.canCoderInvert;

    // TODO Must tune for this robot
    /** Maximum continuous current for the angle motor. */
    public static final int ANGLE_MOTOR_CONTINUOUS_CURRENT_MAX = 25;
    /** Maximum peak current for the angle motor. */
    public static final int ANGLE_MOTOR_PEAK_CURRENT_MAX = 40;
    /** Maximum peak current duration for the angle motor. */
    public static final double ANGLE_MOTOR_PEAK_CURRENT_DURATION = 0.1;
    /** Toggle for limiting the current for the angle motor. */
    public static final boolean SHOULD_CURRENT_LIMIT_ANGLE_MOTOR = true;

    // TODO Must tune for this robot
    /** Maximum continuous current for the drive motor. */
    public static final int DRIVE_MOTOR_CONTINUOUS_CURRENT_MAX = 35;
    /** Maximum peak current for the drive motor. */
    public static final int DRIVE_MOTOR_PEAK_CURRENT_MAX = 60;
    /** Maximum peak current duration for the drive motor. */
    public static final double DRIVE_MOTOR_PEAK_CURRENT_DURATION = 0.1;
    /** Toggle for limiting the current for the drive motor. */
    public static final boolean SHOULD_CURRENT_LIMIT_DRIVE_MOTOR = true;

    // TODO Must tune for this robot
    // https://api.ctr-electronics.com/phoenix/release/java/com/ctre/phoenix/motorcontrol/can/BaseMotorControllerConfiguration.html#openloopRamp
    /** Seconds to go from 0 to full in open loop. Tune while driving on carpet. */
    public static final double OPEN_LOOP_RAMP_DURATION = 0.5;
    /** Seconds to go from 0 to full in closed loop. Tune while driving on carpet. */
    public static final double CLOSED_LOOP_RAMP_DURATION = 0.0;
    // https://github.com/Team364/BaseFalconSwerve/issues/10
    /** Toggle for driving in open loop for teleop. Open loop is not bounded by maximum speed. */
    public static final boolean SHOULD_OPEN_LOOP_IN_TELEOP = true;

    // TODO Must tune for this robot
    /** Angle motor KP. */
    public static final double ANGLE_MOTOR_KP = COTS_MODULE_TYPE.angleKP;
    /** Angle motor KI. */
    public static final double ANGLE_MOTOR_KI = COTS_MODULE_TYPE.angleKI;
    /** Angle motor KD. */
    public static final double ANGLE_MOTOR_KD = COTS_MODULE_TYPE.angleKD;
    /** Angle motor KF. */
    public static final double ANGLE_MOTOR_KF = COTS_MODULE_TYPE.angleKF;

    // TODO Must tune for this robot
    /** Drive motor KP. */
    public static final double DRIVE_MOTOR_KP = 0.05;
    /** Drive motor KI. */
    public static final double DRIVE_MOTOR_KI = 0.0;
    /** Drive motor KD. */
    public static final double DRIVE_MOTOR_KD = 0.0;
    /** Drive motor KF. */
    public static final double DRIVE_MOTOR_KF = 0.0;

    // TODO Must tune for this robot
    /**
     * Drive motor KS. KS is the voltage needed to overcome static friction. Copy these values from
     * the System Identification application.
     */
    public static final double DRIVE_MOTOR_KS = (0.32 / 12);
    /**
     * Drive motor KV. KV is the voltage needed to cruise at a constant velocity. Copy these values
     * from the System Identification application.
     */
    public static final double DRIVE_MOTOR_KV = (1.51 / 12);
    /**
     * Drive motor KA. KA is the voltage needed to induce a given acceleration. Copy these values
     * from the System Identification application.
     */
    public static final double DRIVE_MOTOR_KA = (0.27 / 12);

    /** Maximum linear speed, in meters per second. TODO Tune while driving on carpet. */
    public static final double MAX_SPEED = 5;
    /** Maximum angular speed, in radians per second. TODO Tune while driving on carpet. */
    public static final double MAX_ANGULAR_SPEED =
        MAX_SPEED / Math.hypot(WHEEL_BASE / 2, TRACK_WIDTH / 2);

    /** Theta (rotation) controller KP. */
    public static final double THETA_CONTROLLER_KP = 1.7 * (MAX_ANGULAR_SPEED / Math.PI);
    /** Theta (rotation) controller KI. */
    public static final double THETA_CONTROLLER_KI = 0;
    /** Theta (rotation) controller KD. */
    public static final double THETA_CONTROLLER_KD = 0.1 * (MAX_ANGULAR_SPEED / Math.PI);
    /** Theta (rotation) controller deadband. */
    public static final double THETA_CONTROLLER_TOLERANCE = Units.degreesToRadians(2);

    /**
     * Mode to enter when the motor is "neutral." TODO Check with the Lead Mentors to decide this
     * behavior.
     */
    public static final NeutralMode ANGLE_MOTOR_NEUTRAL_MODE = NeutralMode.Coast;

    /**
     * Mode to enter when the motor is "neutral." TODO Check with the Lead Mentors to decide this
     * behavior.
     */
    public static final NeutralMode DRIVE_MOTOR_NEUTRAL_MODE = NeutralMode.Coast;

    /**
     * Number of seconds to before entering cross formation. Blocks velocity input for this period
     * to allow deceleration before fully stopping.
     */
    public static final double CROSS_FORMATION_DELAY = 0.15;

    /** Front Left Module */
    public static final class FRONT_LEFT_MODULE {
      /** CAN ID of the drive motor. */
      public static final int DRIVE_MOTOR_ID = 3;
      /** CAN ID of the angle motor. */
      public static final int ANGLE_MOTOR_ID = 1;
      /** CAN ID of the CANCoder. */
      public static final int CANCODER_ID = 2;
      /** Difference between the CANCoder angle and module angle. */
      public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(335.43);
      /** Angle to return to when stopped. */
      public static final Rotation2d ANGLE_STOP = Rotation2d.fromDegrees(45);
      /** FIXME */
      public static final SwerveModuleConfig CONFIG =
          new SwerveModuleConfig(
              DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET, ANGLE_STOP);
    }

    /** Front Right Module */
    public static final class FRONT_RIGHT_MODULE {
      /** CAN ID of the drive motor. */
      public static final int DRIVE_MOTOR_ID = 6;
      /** CAN ID of the angle motor. */
      public static final int ANGLE_MOTOR_ID = 4;
      /** CAN ID of the CANCoder. */
      public static final int CANCODER_ID = 5;
      /** Difference between the CANCoder angle and module angle. */
      public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(253.74);
      /** Angle to return to when stopped. */
      public static final Rotation2d ANGLE_STOP = Rotation2d.fromDegrees(-45);
      /** FIXME */
      public static final SwerveModuleConfig CONFIG =
          new SwerveModuleConfig(
              DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET, ANGLE_STOP);
    }

    /** Back Left Module */
    public static final class BACK_LEFT_MODULE {
      /** CAN ID of the drive motor. */
      public static final int DRIVE_MOTOR_ID = 13;
      /** CAN ID of the angle motor. */
      public static final int ANGLE_MOTOR_ID = 11;
      /** CAN ID of the CANCoder. */
      public static final int CANCODER_ID = 12;
      /** Difference between the CANCoder angle and module angle. */
      public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(162.15);
      /** Angle to return to when stopped. */
      public static final Rotation2d ANGLE_STOP = Rotation2d.fromDegrees(-45);
      /** FIXME */
      public static final SwerveModuleConfig CONFIG =
          new SwerveModuleConfig(
              DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET, ANGLE_STOP);
    }

    /** Back Right Module */
    public static final class BACK_RIGHT_MODULE {
      /** CAN ID of the drive motor. */
      public static final int DRIVE_MOTOR_ID = 10;
      /** CAN ID of the angle motor. */
      public static final int ANGLE_MOTOR_ID = 8;
      /** CAN ID of the CANCoder. */
      public static final int CANCODER_ID = 9;
      /** Difference between the CANCoder angle and module angle. */
      public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(186.715);
      /** Angle to return to when stopped. */
      public static final Rotation2d ANGLE_STOP = Rotation2d.fromDegrees(45);
      /** FIXME */
      public static final SwerveModuleConfig CONFIG =
          new SwerveModuleConfig(
              DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET, ANGLE_STOP);
    }
  }

  public static final class Auto {
    /**
     * List of commands to run whenever a marker is reached. See
     * https://github.com/mjansen4857/pathplanner/wiki/Editor-Modes#marker-card for usage.
     */
    public static final Map<String, Command> EVENT_MAP = new HashMap<>();

    static {
      EVENT_MAP.put("DoNothing", new DoNothingCommand());
    }

    /** PID constants for translation controller. */
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(1.0, 0.0, 0.0);
    /** PID constants for rotation (theta) controller. */
    public static final PIDConstants ROTATION_PID = new PIDConstants(1.0, 0.0, 0.0);

    /**
     * Constrains generated paths with maximum velocity of 3.0 meters per second and maximum
     * acceleration of 3.0 meters per second per second.
     */
    public static final PathConstraints CONSTRAINTS = new PathConstraints(3.0, 3.0);
  }

  public static final class Lighting {
    /** CAN ID of the CANdle. */
    public static final int CANDLE_ID = 40; // TODO Configure

    /** Hex code for black (no status). */
    public static final Color COLOR_BLACK = Color.kBlack;
    /** Hex code for yellow (cone). */
    public static final Color COLOR_YELLOW = Color.kYellow;
    /** Hex code for purple (cube). */
    // public static final Color COLOR_PURPLE = Color.kPurple;
    public static final Color COLOR_PURPLE = Color.kMediumPurple;
    /** Hex code for red (not aligned). */
    public static final Color COLOR_RED = Color.kRed;
    /** Hex code for green (aligned). */
    public static final Color COLOR_GREEN = Color.kGreen;
  }

  public static final class Arm {

    /**
     * Name of the CAN bus for all arm devices. By default, "rio" or "" selects the CAN bus
     * beginning at the RoboRio. If connecting using a CANivore, use Phoenix Tuner to view the CAN
     * bus name.
     */
    public static final String CANBUS_NAME = "rio";

    public static final class Extension {
      /**
       * CAN ID of the extension motor. Locate the correct motor in Phoenix Tuner using the Blink
       * button, then copy the ID of the motor to this constant.
       */
      public static final int MOTOR_ID = 3;

      /**
       * Maximum continuous current for the extension motor. This is the current that the motor will
       * be held at if the current limit is exceeded.
       */
      public static final double CONTINUOUS_CURRENT_MAX = 0; // TODO
      /**
       * Maximum peak current for the extension motor. If this current is exceeded for the specified
       * duration, current will be reduced.
       */
      public static final double PEAK_CURRENT_MAX = 0; // TODO
      /**
       * Maximum peak current duration for the extension motor. If the specified current is exceeded
       * for this duration, current will be reduced.
       */
      public static final double PEAK_CURRENT_DURATION = 0; // TODO
      /**
       * Toggle for limiting the current for the extension motor. If true, current will be limited
       * using the specified parameters.
       */
      public static final boolean SHOULD_CURRENT_LIMIT = false; // TODO

      /** Extension motor KP. Applies this many volts per meter of error. */
      public static final double KP = 0; // TODO
      /** Extension motor KI. */
      public static final double KI = 0; // TODO
      /** Extension motor KD. */
      public static final double KD = 0; // TODO
      /** Contrains maximum velocity to 1 m/s and maximum accleration to 1 m/s/s. */
      public static final TrapezoidProfile.Constraints CONSTRAINTS =
          new TrapezoidProfile.Constraints(1, 1);

      /**
       * KG is the voltage needed to overcome gravity. Used to add feedforward voltage to the PID
       * output.
       */
      public static final double KG = 0; // TODO
      /**
       * KS is the voltage needed to overcome static friction. Used to add feedforward voltage to
       * the PID output.
       */
      public static final double KS = 0; // TODO
      /**
       * KV is the voltage needed to cruise at a constant velocity. Used to add feedforward voltage
       * to the PID output.
       */
      public static final double KV = 0; // TODO
      /**
       * KA is the voltage needed to induce a given acceleration. Used to add feedforward voltage to
       * the PID output.
       */
      public static final double KA = 0; // TODO

      /** The maximum voltage that can be applied to motors. */
      public static final int MAX_VOLTAGE = 0;

      /**
       * Toggle for if the extension motor should be inverted. Ensures that positive values cause
       * the arm to extend and that negative values cause the arm to retract.
       */
      public static final boolean SHOULD_INVERT_MOTOR = false;

      /**
       * Mode that the motor enters when no effort is applied. Brake mode makes it more difficult to
       * turn the shaft, while neutral mode allows the shaft the turn freely.
       */
      public static final NeutralMode MOTOR_NEUTRAL_MODE = NeutralMode.Brake;

      /**
       * The difference in arm length caused by one full rotation of the spool, meaured in meters.
       * Essentially, the effective "circumference" of the spool. Used for calculating the
       * displacement per encoder tick.
       */
      public static final double LENGTH_PER_ROTATION = Units.inchesToMeters(1) * Math.PI;
      /**
       * The gear ratio between the extension motor and the spool. Used for calculating the
       * displacement per encoder tick.
       */
      public static final double GEAR_RATIO = 15.34; // TODO

      /**
       * Channel on the Pneumatics Hub for the brake solenoid. Used for signalling the solenoid to
       * engage and disengage the brake on the gearbox.
       */
      public static final int BRAKE_CHANNEL = 9;

      /**
       * The speed for manually extending the arm. Used for manual driving of the arm, as a bypass
       * for the PID control.
       */
      public static final double MANUAL_EXTEND_SPEED = 0.8;

      /**
       * The speed for manually retracting the arm. Used for manual driving of the arm, as a bypass
       * for the PID control.
       */
      public static final double MANUAL_RETRACT_SPEED = -0.8;

      public static final double MAX_EXTENSION_LENGTH = 1.3;

      public static final double MIN_EXTENSION_LENGTH = 0.0;
    }

    public static final class Rotation {

      /**
       * CAN ID of the rotation motor. Locate the correct motor in Phoenix Tuner using the Blink
       * button, then copy the ID of the motor to this constant.
       */
      public static final int MOTOR_ID = 2;
      /**
       * CAN ID of the rotation CANcoder. Locate the correct CANcoder in Phoenix Tuner using the
       * Blink button, then copy the ID of the CANcoder to this constant.
       */
      public static final int CANCODER_ID = 1;

      /**
       * Maximum continuous current for the extension motor. This is the current that the motor will
       * be held at if the current limit is exceeded.
       */
      public static final double CONTINUOUS_CURRENT_MAX = 0; // TODO
      /**
       * Maximum peak current for the extension motor. If this current is exceeded for the specified
       * duration, current will be reduced.
       */
      public static final double PEAK_CURRENT_MAX = 0; // TODO
      /**
       * Maximum peak current duration for the extension motor. If the specified current is exceeded
       * for this duration, current will be reduced.
       */
      public static final double PEAK_CURRENT_DURATION = 0; // TODO
      /**
       * Toggle for limiting the current for the extension motor. If true, current will be limited
       * using the specified parameters.
       */
      public static final boolean SHOULD_CURRENT_LIMIT = false; // TODO

      /** Rotation motor KP. Applies this many volts per degree of error. */
      public static final double KP = 0.2; // TODO
      /** Rotation motor KI. */
      public static final double KI = 0; // TODO
      /** Rotation motor KD. */
      public static final double KD = 0; // TODO
      /** Contrains maximum velocity to 1 deg/s and maximum accleration to 1 deg/s/s. */
      public static final TrapezoidProfile.Constraints CONSTRAINTS =
          new TrapezoidProfile.Constraints(80, 10);

      /**
       * KG is the voltage needed to overcome gravity. Used to add feedforward voltage to the PID
       * output.
       */
      public static final double KG = 0; // TODO
      /**
       * KS is the voltage needed to overcome static friction. Used to add feedforward voltage to
       * the PID output.
       */
      public static final double KS = 0.25; // TODO
      /**
       * KV is the voltage needed to cruise at a constant velocity. Used to add feedforward voltage
       * to the PID output.
       */
      public static final double KV = 0; // TODO
      /**
       * KA is the voltage needed to induce a given acceleration. Used to add feedforward voltage to
       * the PID output.
       */
      public static final double KA = 0; // TODO

      /** The maximum voltage that can be applied to motors. */
      public static final int MAX_VOLTAGE = 6;

      /**
       * Toggle for if the CANCoder should be inverted. Ensures that positive angles are
       * counter-clockwise and negative angles are clockwise.
       */
      public static final boolean SHOULD_INVERT_CANCODER = false;

      /**
       * Toggle for if the rotation motor should be inverted. Ensures that positive motor values
       * cause upwards movement and negative motor values cause downwards movement.
       */
      public static final boolean SHOULD_INVERT_MOTOR = true;

      /**
       * Mode that the motor enters when no effort is applied. Brake mode makes it more difficult to
       * spin the shaft, while neutral mode allows the shaft the spin freely.
       */
      public static final NeutralMode MOTOR_NEUTRAL_MODE = NeutralMode.Brake;

      /**
       * The gear ratio between the rotation motor and the arm. Used for calculating the
       * displacement per encoder tick.
       */
      public static final double GEAR_RATIO = 24.1666; // TODO

      /**
       * The difference between 0 degrees on the CANCoder and 0 degrees on the mechanism. Used for
       * keeping a constant frame of reference for all calculations, where 0 degrees is parallel
       * with the ground and perpendicular with the superstructure.
       */
      public static final double CANCODER_OFFSET = 0; // TODO

      /**
       * The minimum angle that the arm can rotate to. This value is measured in degrees, and
       * represents the lower bound that the arm will never cross.
       */
      public static final double MIN_ANGLE = -300;

      /**
       * The maximum angle that the arm can rotate to. This value is measured in degrees, and
       * represents the upper bound that the arm will never cross.
       */
      public static final double MAX_ANGLE = 0;

      /**
       * Channel on the Pneumatics Hub for the brake solenoid. Used for signalling the solenoid to
       * engage and disengage the brake on the gearbox.
       */
      public static final int BRAKE_CHANNEL = 10;

      /** The speed for manually raising the arm. */
      public static final double MANUAL_RAISE_SPEED = 0.1;

      /** The speed for manually lowering the arm. */
      public static final double MANUAL_LOWER_SPEED = -0.1;
    }

    public static final class States {
      public static final ArmState STOWED = new ArmState(0, Rotation2d.fromDegrees(0));
      public static final ArmState FLOOR = new ArmState(0, Rotation2d.fromDegrees(-300)); // TODO
      public static final ArmState MIDDLE = new ArmState(0, Rotation2d.fromDegrees(0)); // TODO
      public static final ArmState TOP = new ArmState(0, Rotation2d.fromDegrees(0)); // TODO
      public static final ArmState SUBSTATION = new ArmState(0, Rotation2d.fromDegrees(0)); // TODO
    }
  }

  public static final class Claw {
    public static final int CHANNEL = 8;
  }
}
