package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.*;
import frc.robot.swerve.Swerve;
import frc.robot.swerve.TeleopDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final Joystick driverController = new Joystick(Constants.Driver.CONTROLLER_PORT);

  /* Driver Buttons */
  private final Trigger zeroGyro =
      new Trigger(() -> driverController.getRawButton(Constants.Driver.BUTTON_ZERO_GYRO.value));
  private final Trigger robotCentric =
      new Trigger(() -> driverController.getRawButton(Constants.Driver.BUTTON_ROBOT_CENTRIC.value));
  private final Trigger turbo =
      new Trigger(
          () ->
              driverController.getRawAxis(Constants.Driver.AXIS_TURBO_MODE.value)
                  > Constants.Driver.TRIGGER_THRESHOLD);

  /* Subsystems */
  private final Swerve swerve = new Swerve();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerve.setDefaultCommand(
        new TeleopDrive(
            swerve,
            () -> -driverController.getRawAxis(Constants.Driver.AXIS_TRANSLATION.value),
            () -> -driverController.getRawAxis(Constants.Driver.AXIS_STRAFE.value),
            () -> -driverController.getRawAxis(Constants.Driver.AXIS_ROTATION.value),
            () -> robotCentric.getAsBoolean()));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyro.onTrue(swerve.zeroGyro());
    turbo.onTrue(swerve.enableTurbo()).onFalse(swerve.disableTurbo());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new exampleAuto(swerve);
  }
}
