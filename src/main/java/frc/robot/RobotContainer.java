package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.auto.Autos;
import frc.robot.superstructure.Claw;
import frc.robot.superstructure.ExtensionController;
import frc.robot.superstructure.RotationController;
import frc.robot.superstructure.commands.controlled.DumbRotate;
import frc.robot.superstructure.commands.manual.SafeExtend;
import frc.robot.superstructure.commands.manual.SafeLower;
import frc.robot.superstructure.commands.manual.SafeRaise;
import frc.robot.superstructure.commands.manual.SafeRetract;
import frc.robot.swerve.Swerve;
import frc.robot.swerve.TeleopDrive;
import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final Swerve m_swerve = new Swerve(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final Claw m_claw = new Claw();
  private final RotationController m_rotationController = new RotationController();
  private final ExtensionController m_extensionController =
      new ExtensionController(m_rotationController);

  // Controllers
  private final CommandXboxController m_driver = new CommandXboxController(0);
  private final CommandXboxController m_manipulator =
      new CommandXboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    setDefaultCommands();
    configureButtonBindings();
    configureTriggers();

    SmartDashboard.putData(m_rotationController);
  }

  /**
   * The default command will be automatically scheduled when no other commands are scheduled that
   * require the subsystem.
   */
  private void setDefaultCommands() {

    TeleopDrive teleopDrive =
        new TeleopDrive(
            m_swerve,
            () ->
                MathUtil.applyDeadband(
                    m_driver.getRawAxis(XboxController.Axis.kLeftY.value),
                    Constants.Driver.DEADBAND),
            () ->
                MathUtil.applyDeadband(
                    m_driver.getRawAxis(XboxController.Axis.kLeftX.value),
                    Constants.Driver.DEADBAND),
            () ->
                MathUtil.applyDeadband(
                    m_driver.getRawAxis(XboxController.Axis.kRightX.value),
                    Constants.Driver.DEADBAND),
            () -> true, // Always drive field-oriented
            false,
            false);

    m_swerve.setDefaultCommand(teleopDrive);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_driver.y().onTrue(new InstantCommand(m_swerve::zeroGyro));//.onTrue(new PrintCommand("Zero gyro"));
    m_driver.x().onTrue(new InstantCommand(m_swerve::lock));//.onTrue(new PrintCommand("Lock modules"));

    m_manipulator.leftTrigger().onTrue(new InstantCommand(m_claw::close));//.onTrue(new PrintCommand("Close claw"));
    m_manipulator.rightTrigger().onTrue(new InstantCommand(m_claw::open));//.onTrue(new PrintCommand("Open claw"));

    m_manipulator.axisLessThan(XboxController.Axis.kRightY.value, -Constants.Driver.DEADBAND).whileTrue(new SafeExtend(m_extensionController));//.onTrue(new PrintCommand("Extending"));
    m_manipulator.axisGreaterThan(XboxController.Axis.kRightY.value, Constants.Driver.DEADBAND).whileTrue(new SafeRetract(m_extensionController));//.onTrue(new PrintCommand("Retracting"));

    m_manipulator.axisLessThan(XboxController.Axis.kLeftY.value, -Constants.Driver.DEADBAND).whileTrue(new SafeRaise(m_rotationController));//.onTrue(new PrintCommand("Raising"));
    m_manipulator.axisGreaterThan(XboxController.Axis.kLeftY.value, Constants.Driver.DEADBAND).whileTrue(new SafeLower(m_rotationController));//.onTrue(new PrintCommand("Lowering"));

    m_manipulator.x().whileTrue(new DumbRotate(m_rotationController, Constants.Arm.Angles.FLOOR));//.onTrue(new PrintCommand("Rotating to floor"));
    m_manipulator.a().whileTrue(new DumbRotate(m_rotationController, 0));//.onTrue(new PrintCommand("Rotating to stowed"));
    m_manipulator.y().whileTrue(new DumbRotate(m_rotationController, -100));//.onTrue(new PrintCommand("Rotating to top"));
    m_manipulator.b().whileTrue(new DumbRotate(m_rotationController, Constants.Arm.Angles.SUBSTATION));//.onTrue(new PrintCommand("Rotating to substation"));
  }

  private void configureTriggers() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return Autos.extendDropAndRetractAndBackup(
        m_extensionController, m_rotationController, m_claw, m_swerve);
  }
}