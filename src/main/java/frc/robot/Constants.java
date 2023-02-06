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
import frc.lib.swerve.COTSFalconSwerveConstants;
import frc.lib.swerve.SwerveModuleConstants;

public final class Constants {

  public static final class OperatorInput {
    /** Minimum stick displacement to register movement. */
    public static final double STICK_DEADBAND = 0.2;
  }

  public static final class Swerve {
    /** CAN ID of the Pigeon2. */
    public static final int PIGEON_ID = 7;
    /** Toggle for if the Pigeon2 is CCW+ CW-. */
    public static final boolean SHOULD_INVERT_GYRO = false;

    /** Gear ratio of the drive motor. */
    public static final double DRIVE_MOTOR_GEAR_RATIO_SPEC =
        COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L3;
    /** Swerve module type. */
    public static final COTSFalconSwerveConstants COTS_MODULE_TYPE =
        COTSFalconSwerveConstants.SDSMK4i(DRIVE_MOTOR_GEAR_RATIO_SPEC);

    /** Center-to-center distance of left and right modules, in meters. */
    public static final double TRACK_WIDTH = Units.inchesToMeters(22.75);
    /** Center-to-center distance of front and rear modules, in meters. */
    public static final double WHEEL_BASE = Units.inchesToMeters(22.75);
    /** Cirumference of the wheel (including tread) in meters. */
    public static final double WHEEL_CIRCUMFERENCE = COTS_MODULE_TYPE.wheelCircumference;

    /** Inverse kinematics helper class. */
    public static final SwerveDriveKinematics SWERVE_KINEMATICS =
        new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

    /** Gear ratio for the drive motor. */
    public static final double DRIVE_MOTOR_GEAR_RATIO = COTS_MODULE_TYPE.driveGearRatio;
    /** Gear ratio for the angle motor. */
    public static final double ANGLE_MOTOR_GEAR_RATIO = COTS_MODULE_TYPE.angleGearRatio;

    /** Toggle for angle motor CCW+. */
    public static final boolean SHOULD_INVERT_ANGLE_MOTOR = COTS_MODULE_TYPE.angleMotorInvert;
    /** Toggle for drive motor CCW+. */
    public static final boolean SHOULD_INVERT_DRIVE_MOTOR = COTS_MODULE_TYPE.driveMotorInvert;
    /** Toggle for CANCoder CCW+. */
    public static final boolean SHOULD_INVERT_CANCODER = COTS_MODULE_TYPE.canCoderInvert;

    /** Maximum continuous current for the angle motor. */
    public static final int ANGLE_MOTOR_CONTINUOUS_CURRENT_MAX = 25;
    /** Maximum peak current for the angle motor. */
    public static final int ANGLE_MOTOR_PEAK_CURRENT_MAX = 40;
    /** Maximum peak current duration for the angle motor. */
    public static final double ANGLE_MOTOR_PEAK_CURRENT_DURATION = 0.1;
    /** Toggle for limiting the current for the angle motor. */
    public static final boolean SHOULD_CURRENT_LIMIT_ANGLE_MOTOR = true;

    /** Maximum continuous current for the drive motor. */
    public static final int DRIVE_MOTOR_CONTINUOUS_CURRENT_MAX = 35;
    /** Maximum peak current for the drive motor. */
    public static final int DRIVE_MOTOR_PEAK_CURRENT_MAX = 60;
    /** Maximum peak current duration for the drive motor. */
    public static final double DRIVE_MOTOR_PEAK_CURRENT_DURATION = 0.1;
    /** Toggle for limiting the current for the drive motor. */
    public static final boolean SHOULD_CURRENT_LIMIT_DRIVE_MOTOR = true;

    // https://api.ctr-electronics.com/phoenix/release/java/com/ctre/phoenix/motorcontrol/can/BaseMotorControllerConfiguration.html#openloopRamp
    /** Seconds to go from 0 to full in open loop. */
    public static final double OPEN_LOOP_RAMP_DURATION = 0.5;
    /** Seconds to go from 0 to full in closed loop. */
    public static final double CLOSED_LOOP_RAMP_DURATION = 0.0;

    /** Angle motor KP. */
    public static final double ANGLE_MOTOR_KP = COTS_MODULE_TYPE.angleKP;
    /** Angle motor KI. */
    public static final double ANGLE_MOTOR_KI = COTS_MODULE_TYPE.angleKI;
    /** Angle motor KD. */
    public static final double ANGLE_MOTOR_KD = COTS_MODULE_TYPE.angleKD;
    /** Angle motor KF. */
    public static final double ANGLE_MOTOR_KF = COTS_MODULE_TYPE.angleKF;

    /** Drive motor KP. */
    public static final double DRIVE_MOTOR_KP = 0.05;
    /** Drive motor KI. */
    public static final double DRIVE_MOTOR_KI = 0.0;
    /** Drive motor KD. */
    public static final double DRIVE_MOTOR_KD = 0.0;
    /** Drive motor KF. */
    public static final double DRIVE_MOTOR_KF = 0.0;

    // TODO Must tune for this robot
    /** Drive motor KS. */
    public static final double DRIVE_MOTOR_KS = (0.32 / 12);
    /** Drive motor KV. */
    public static final double DRIVE_MOTOR_KV = (1.51 / 12);
    /** Drive motor KA. */
    public static final double DRIVE_MOTOR_KA = (0.27 / 12);

    // TODO Must tune for this robot
    /** Maximum linear speed, in meters per second. */
    public static final double LINEAR_SPEED_MAX = 4.5;
    /** Maximum angular speed, in meters per second. */
    public static final double ANGULAR_SPEED_MAX = 10.0;

    // https://api.ctr-electronics.com/phoenix/release/java/com/ctre/phoenix/motorcontrol/NeutralMode.html
    /** Mode to enter when the motor is "neutral." */
    public static final NeutralMode ANGLE_MOTOR_NEUTRAL_MODE = NeutralMode.Coast;
    /** Mode to enter when the motor is "neutral." */
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

    /** Hex code for black. */
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
}
