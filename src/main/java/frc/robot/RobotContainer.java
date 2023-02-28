package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autos.exampleAuto;
import frc.robot.swerve.Swerve;
import frc.robot.swerve.TeleopDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final Swerve m_swerve = new Swerve();

  // Controllers
  private final Joystick m_driverController = new Joystick(Constants.Driver.CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    setDefaultCommands();
    configureButtonBindings();
    configureTriggers();
  }

  /**
   * The default command will be automatically scheduled when no other commands are scheduled that
   * require the subsystem.
   */
  private void setDefaultCommands() {
    m_swerve.setDefaultCommand(
        new TeleopDrive(
            m_swerve,
            () -> -MathUtil.applyDeadband(m_driverController.getRawAxis(Constants.Driver.AXIS_TRANSLATION.value), Constants.Driver.DEADBAND),
            () -> -MathUtil.applyDeadband(m_driverController.getRawAxis(Constants.Driver.AXIS_STRAFE.value), Constants.Driver.DEADBAND),
            () -> -MathUtil.applyDeadband(m_driverController.getRawAxis(Constants.Driver.AXIS_ROTATION.value), Constants.Driver.DEADBAND)));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
  }

  private void configureTriggers() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new exampleAuto(m_swerve);
  }
}
