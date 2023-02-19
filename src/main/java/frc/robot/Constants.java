// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.lib.swerve.COTSFalconSwerveConstants;
import frc.lib.swerve.SwerveModuleConstants;

public final class Constants {

  public static final class Driver {
    /** Port in Driver Station for the driver controller. */
    public static final int CONTROLLER_PORT = 1;
    /** Minimum axis displacement to register movement. */
    public static final double DEADBAND = 0.2;
    /** Axis for forward-backward movement. */
    public static final int AXIS_TRANSLATION = XboxController.Axis.kLeftY.value;
    /** Axis for left-right movement. */
    public static final int AXIS_STRAFE = XboxController.Axis.kLeftX.value;
    /** Axis for rotation. */
    public static final int AXIS_ROTATION = XboxController.Axis.kRightX.value;
    /** Button for zeroing the gyro. */
    public static final int BUTTON_ZERO_GYRO = XboxController.Button.kY.value;
    /** Button for driving in robot-centric. */
    public static final int BUTTON_ROBOT_CENTRIC = XboxController.Button.kLeftBumper.value;
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
    public static final SwerveDriveKinematics SWERVE_KINEMATICS =
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
    // https://docs.wpilib.org/en/stable/docs/software/pathplanning/system-identification/introduction.html
    // V (volts) = KS (volts) + KV (volts / velocity) * d' (velocity) + KA (volts / acceleration) *
    // d'' (acceleration)
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

    // TODO Must tune for this robot
    /** Maximum linear speed, in meters per second. Tune while driving on carpet. */
    public static final double LINEAR_SPEED_MAX = 4.5;
    /** Maximum angular speed, in meters per second. Tune while driving on carpet. */
    public static final double ANGULAR_SPEED_MAX = 10.0;

    // https://api.ctr-electronics.com/phoenix/release/java/com/ctre/phoenix/motorcontrol/NeutralMode.html
    /**
     * Mode to enter when the motor is "neutral." Check with the Lead Mentors to decide this
     * behavior.
     */
    public static final NeutralMode ANGLE_MOTOR_NEUTRAL_MODE = NeutralMode.Coast;
    /**
     * Mode to enter when the motor is "neutral." Check with the Lead Mentors to decide this
     * behavior.
     */
    public static final NeutralMode DRIVE_MOTOR_NEUTRAL_MODE = NeutralMode.Coast;

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
      /** FIXME */
      public static final SwerveModuleConstants CONSTANTS =
          new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
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
      /** FIXME */
      public static final SwerveModuleConstants CONSTANTS =
          new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
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
      /** FIXME */
      public static final SwerveModuleConstants CONSTANTS =
          new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
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
      /** FIXME */
      public static final SwerveModuleConstants CONSTANTS =
          new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
    }
  }

  public static final class Auto {
    /** Maximum linear speed during auto, in meters per second. */
    public static final double LINEAR_SPEED_MAX = 3;
    /** Maximum linear acceleration during auto, in meters per second squared. */
    public static final double LINEAR_ACCELERATION_MAX = 3;
    /** Maximum angular speed during auto, in radians per second. */
    public static final double ANGULAR_SPEED_MAX = Math.PI;
    /** Maximum angular acceleration during auto, in radians per second squared. */
    public static final double ANGULAR_ACCELERATION_MAX = Math.PI;

    /** X position (translation) controller KP. */
    public static final double X_CONTROLLER_KP = 1;
    /** Y position (translation) controller KP. */
    public static final double Y_CONTROLLER_KP = 1;
    /** Theta (rotation) controller KP. */
    public static final double THETA_CONTROLLER_KP = 1;

    /** Constraints for the theta (rotation) controller. */
    public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
        new TrapezoidProfile.Constraints(ANGULAR_SPEED_MAX, ANGULAR_ACCELERATION_MAX);
  }

  public static final class Lighting {
    /** CAN ID of the CANdle. */
    public static final int CANDLE_ID = 40; // TODO Configure

    /** Hex code for black (no status). */
    public static final String COLOR_BLACK = "#000000";
    /** Hex code for yellow (cone). */
    public static final String COLOR_YELLOW = "#ffe606";
    /** Hex code for purple (cube). */
    public static final String COLOR_PURPLE = "#e330ff";
    /** Hex code for red (not aligned). */
    public static final String COLOR_RED = "#ff0000";
    /** Hex code for green (aligned). */
    public static final String COLOR_GREEN = "#00ff00";
  }

  public static final class Arm {
    /**
     * Name of the CAN bus for all arm devices. By default, "rio" or "" selects the CAN bus
     * beginning at the RoboRio. If connecting using a CANivore, use Phoenix Tuner to view the CAN
     * bus name.
     */
    public static final String CANBUS_NAME = "rio";

    /**
     * CAN ID of the rotation motor. Locate the correct motor in Phoenix Tuner using the Blink
     * button, then copy the ID of the motor to this constant.
     */
    public static final int ROTATION_MOTOR_CAN_ID = 0; // TODO
    /**
     * CAN ID of the extension motor. Locate the correct motor in Phoenix Tuner using the Blink
     * button, then copy the ID of the motor to this constant.
     */
    public static final int EXTENSION_MOTOR_CAN_ID = 0; // TODO
    /**
     * CAN ID of the rotation CANcoder. Locate the correct CANcoder in Phoenix Tuner using the Blink
     * button, then copy the ID of the CANcoder to this constant.
     */
    public static final int ROTATION_CANCODER_CAN_ID = 0; // TODO

    /** Maximum continuous current for the rotation motor. */
    public static final double ROTATION_MOTOR_CONTINUOUS_CURRENT_MAX = 0; // TODO
    /** Maximum peak current for the rotation motor. */
    public static final double ROTATION_MOTOR_PEAK_CURRENT_MAX = 0; // TODO
    /** Maximum peak current duration for the rotation motor. */
    public static final double ROTATION_MOTOR_PEAK_CURRENT_DURATION = 0; // TODO
    /** Toggle for limiting the current for the rotation motor. */
    public static final boolean SHOULD_CURRENT_LIMIT_ROTATION_MOTOR = false; // TODO

    /** Rotation motor KP. */
    public static final double ROTATION_MOTOR_KP = 0; // TODO
    /** Rotation motor KI. */
    public static final double ROTATION_MOTOR_KI = 0; // TODO
    /** Rotation motor KD. */
    public static final double ROTATION_MOTOR_KD = 0; // TODO
    /** Rotation motor KF. */
    public static final double ROTATION_MOTOR_KF = 0; // TODO

    /** Maximum continuous current for the extension motor. */
    public static final double EXTENSION_MOTOR_CONTINUOUS_CURRENT_MAX = 0; // TODO
    /** Maximum peak current for the extension motor. */
    public static final double EXTENSION_MOTOR_PEAK_CURRENT_MAX = 0; // TODO
    /** Maximum peak current duration for the extension motor. */
    public static final double EXTENSION_MOTOR_PEAK_CURRENT_DURATION = 0; // TODO
    /** Toggle for limiting the current for the extension motor. */
    public static final boolean SHOULD_CURRENT_LIMIT_EXTENSION_MOTOR = false; // TODO

    /** Extension motor KP. */
    public static final double EXTENSION_MOTOR_KP = 0; // TODO
    /** Extension motor KI. */
    public static final double EXTENSION_MOTOR_KI = 0; // TODO
    /** Extension motor KD. */
    public static final double EXTENSION_MOTOR_KD = 0; // TODO
    /** Extension motor KF. */
    public static final double EXTENSION_MOTOR_KF = 0; // TODO

    /** Toggle for if the CANCoder should be inverted. Ensure that CCW+ CW-. */
    public static final boolean SHOULD_INVERT_CANCODER = false; // TODO

    /** Toggle for if the rotation motor should be inverted. Ensure that CCW+ CW-. */
    public static final boolean SHOULD_INVERT_ROTATION_MOTOR = false; // TODO

    /**
     * Mode to enter when the motor is "neutral." Check with the Lead Mentors to decide this
     * behavior.
     */
    public static final NeutralMode ROTATION_MOTOR_NEUTRAL_MODE = NeutralMode.Brake; // TODO

    /**
     * Toggle for if the extension motor should be inverted. Ensure that positive values cause the
     * arm to extend and that negative values cause the arm to retract.
     */
    public static final boolean SHOULD_INVERT_EXTENSION_MOTOR = false; // TODO

    /**
     * Mode to enter when the motor is "neutral." Check with the Lead Mentors to decide this
     * behavior.
     */
    public static final NeutralMode EXTENSION_MOTOR_NEUTRAL_MODE = NeutralMode.Brake; // TODO
  }
}
