// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.lib.ArmState;
import java.util.HashMap;
import java.util.Map;
import swervelib.math.Matter;

public final class Constants {

  /** The CAN ID of the Pneumatics Hub. */
  public static final int PNEUMATICS_HUB_ID = 30;

  /** The numbers of seconds per full sensor & output cycle. Used to limit the chassis velocity. */
  public static final double LOOP_TIME = 0.02; // 20 milliseconds

  /** The mass of the robot in kilograms. Used to limit the chassis velocity to prevent tipping. */
  public static final double ROBOT_MASS = 0; // TODO

  /** The center of matter on the chassis. */
  public static final Matter CHASSIS =
      new Matter(new Translation3d(0.0, 0.0, 0.0), ROBOT_MASS); // TODO

  public static final class Auto {
    /**
     * List of commands to run whenever a marker is reached. See
     * https://github.com/mjansen4857/pathplanner/wiki/Editor-Modes#marker-card for usage.
     */
    public static final Map<String, Command> EVENT_MAP = new HashMap<>();

    static {
      EVENT_MAP.put("ToTop", new PrintCommand("ToTop"));
      EVENT_MAP.put("ToFloor", new PrintCommand("ToFloor"));
      EVENT_MAP.put("OpenClaw", new PrintCommand("OpenClaw"));
      EVENT_MAP.put("CloseClaw", new PrintCommand("CloseClaw"));
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
      /**
       * The extension tolerance in meters. Used for stopping the PID control when within this
       * threshold of the setpoint.
       */
      public static final double TOLERANCE = 0.1; // TODO

      /** The maximum voltage that can be applied to motors. */
      public static final int MAX_VOLTAGE = 0; // TODO

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
      public static final int BRAKE_CHANNEL = 10;

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
      public static final double KP = 0.0; // TODO
      /** Rotation motor KI. */
      public static final double KI = 0; // TODO
      /** Rotation motor KD. */
      public static final double KD = 0; // TODO

      /**
       * The rotation tolerance in degrees. Used for stopping the PID control when within this
       * tolerance.
       */
      public static final double TOLERANCE = 2; // TODO

      /** The maximum voltage that can be applied to motors. */
      public static final int MAX_VOLTAGE = 0; // TODO

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
      public static final int BRAKE_CHANNEL = 9;

      /** The speed for manually raising the arm. */
      public static final double MANUAL_RAISE_SPEED = 0.1;

      /** The speed for manually lowering the arm. */
      public static final double MANUAL_LOWER_SPEED = -0.1;
    }

    public static final class States {
      public static final ArmState STOWED = new ArmState(0, Rotation2d.fromDegrees(0));
      public static final ArmState FLOOR = new ArmState(0, Rotation2d.fromDegrees(-300));
      public static final ArmState MIDDLE = new ArmState(0, Rotation2d.fromDegrees(0)); // TODO
      public static final ArmState TOP = new ArmState(0, Rotation2d.fromDegrees(0)); // TODO
      public static final ArmState SUBSTATION = new ArmState(0, Rotation2d.fromDegrees(-115));
    }
  }

  public static final class Claw {
    public static final int CHANNEL = 8;
  }
}
