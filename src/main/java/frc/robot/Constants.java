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

  public static final class OI {
    /** Minimum stick displacement to register movement */
    public static final double stickDeadband = 0.2;
  }

  public static final class Swerve {
    /** CAN ID of the Pigeon2 */
    public static final int pigeonID = 7;
    /** Toggle for if the Pigeon2 is CCW+ CW- */
    public static final boolean invertGyro = false;

    /** Gear ratio of the drive motor */
    public static final double driveMotorGearRatio =
        COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L3;
    /** Swerve module type */
    public static final COTSFalconSwerveConstants chosenModule =
        COTSFalconSwerveConstants.SDSMK4i(driveMotorGearRatio);

    /** Center-to-center distance of left and right modules, in meters */
    public static final double trackWidth = Units.inchesToMeters(22.75);
    /** Center-to-center distance of front and rear modules, in meters */
    public static final double wheelBase = Units.inchesToMeters(22.75);
    /** Cirumference of the wheel (including tread) in meters */
    public static final double wheelCircumference = chosenModule.wheelCircumference;

    /** Inverse kinematics helper class */
    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /** Gear ratio for the drive motor */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    /** Gear ratio for the angle motor */
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /** Toggle for angle motor CCW+ */
    public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
    /** Toggle for drive motor CCW+ */
    public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;
    /** Toggle for CANCoder CCW+ */
    public static final boolean canCoderInvert = chosenModule.canCoderInvert;

    /** Maximum continuous current for the angle motor */
    public static final int angleContinuousCurrentLimit = 25;
    /** Maximum peak current for the angle motor */
    public static final int anglePeakCurrentLimit = 40;
    /** Maximum peak current duration for the angle motor */
    public static final double anglePeakCurrentDuration = 0.1;
    /** Toggle for limiting the current for the angle motor */
    public static final boolean angleEnableCurrentLimit = true;

    /** Maximum continuous current for the drive motor */
    public static final int driveContinuousCurrentLimit = 35;
    /** Maximum peak current for the drive motor */
    public static final int drivePeakCurrentLimit = 60;
    /** Maximum peak current duration for the drive motor */
    public static final double drivePeakCurrentDuration = 0.1;
    /** Toggle for limiting the current for the drive motor */
    public static final boolean driveEnableCurrentLimit = true;

    // https://api.ctr-electronics.com/phoenix/release/java/com/ctre/phoenix/motorcontrol/can/BaseMotorControllerConfiguration.html#openloopRamp
    /** Seconds to go from 0 to full in open loop */
    public static final double openLoopRamp = 0.5;
    /** Seconds to go from 0 to full in closed loop */
    public static final double closedLoopRamp = 0.0;

    /** Angle motor KP */
    public static final double angleKP = chosenModule.angleKP;
    /** Angle motor KI */
    public static final double angleKI = chosenModule.angleKI;
    /** Angle motor KD */
    public static final double angleKD = chosenModule.angleKD;
    /** Angle motor KF */
    public static final double angleKF = chosenModule.angleKF;

    /** Drive motor KP */
    public static final double driveKP = 0.05;
    /** Drive motor KI */
    public static final double driveKI = 0.0;
    /** Drive motor KD */
    public static final double driveKD = 0.0;
    /** Drive motor KF */
    public static final double driveKF = 0.0;

    // TODO Must tune for this robot
    /** Drive motor KS */
    public static final double driveKS = (0.32 / 12);
    /** Drive motor KV */
    public static final double driveKV = (1.51 / 12);
    /** Drive motor KA */
    public static final double driveKA = (0.27 / 12);

    // TODO Must tune for this robot
    /** Maximum linear speed, in meters per second */
    public static final double maxLinearSpeed = 4.5;
    /** Maximum angular speed, in meters per second */
    public static final double maxAnglularSpeed = 10.0;

    // https://api.ctr-electronics.com/phoenix/release/java/com/ctre/phoenix/motorcontrol/NeutralMode.html
    /** Mode to enter when the motor is "neutral" */
    public static final NeutralMode angleNeutralMode = NeutralMode.Coast;

    public static final NeutralMode driveNeutralMode = NeutralMode.Coast;

    /** Front Left Module */
    public static final class FrontLeftModule {
      /** CAN ID of the drive motor */
      public static final int driveMotorID = 3;
      /** CAN ID of the angle motor */
      public static final int angleMotorID = 1;
      /** CAN ID of the CANCoder */
      public static final int canCoderID = 2;
      /** Difference between the CANCoder angle and the module's angle */
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(335.43);
      /** FIXME */
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /** Front Right Module */
    public static final class FrontRightModule {
      /** CAN ID of the drive motor */
      public static final int driveMotorID = 6;
      /** CAN ID of the angle motor */
      public static final int angleMotorID = 4;
      /** CAN ID of the CANCoder */
      public static final int canCoderID = 5;
      /** Difference between the CANCoder angle and the module's angle */
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(253.74);
      /** FIXME */
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /** Back Left Module */
    public static final class BackLeftModule {
      /** CAN ID of the drive motor */
      public static final int driveMotorID = 13;
      /** CAN ID of the angle motor */
      public static final int angleMotorID = 11;
      /** CAN ID of the CANCoder */
      public static final int canCoderID = 12;
      /** Difference between the CANCoder angle and the module's angle */
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(162.15);
      /** FIXME */
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /** Back Right Module */
    public static final class BackRightModule {
      /** CAN ID of the drive motor */
      public static final int driveMotorID = 10;
      /** CAN ID of the angle motor */
      public static final int angleMotorID = 8;
      /** CAN ID of the CANCoder */
      public static final int canCoderID = 9;
      /** Difference between the CANCoder angle and the module's angle */
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(186.715);
      /** FIXME */
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
  }

  // TODO Document autonomous constants
  public static final class Auto {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class Lighting {
    public static final int CANDLE_ID = 40; // FIXME Configure

    public static final String COLOR_BLACK = "#000000";
    public static final String COLOR_YELLOW = "#ffe606";
    public static final String COLOR_PURPLE = "#e330ff";
    public static final String COLOR_RED = "#ff0000";
    public static final String COLOR_GREEN = "#00ff00";
  }
}
