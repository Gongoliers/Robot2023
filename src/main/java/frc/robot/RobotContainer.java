package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

import com.thegongoliers.commands.DoNothingCommand;

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
  private final XboxController m_driver = new XboxController(Constants.Driver.CONTROLLER_PORT);
  private final XboxController m_manipulator =
      new XboxController(Constants.Manipulator.CONTROLLER_PORT);

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
                    m_driver.getRawAxis(Constants.Driver.LEFT_VERTICAL_AXIS.value),
                    Constants.Driver.DEADBAND),
            () ->
                MathUtil.applyDeadband(
                    m_driver.getRawAxis(Constants.Driver.LEFT_HORIZONTAL_AXIS.value),
                    Constants.Driver.DEADBAND),
            () ->
                MathUtil.applyDeadband(
                    m_driver.getRawAxis(Constants.Driver.RIGHT_HORIZONTAL_AXIS.value),
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
    new Trigger(() -> m_driver.getRawButton(Constants.Driver.ZERO_GYRO_BUTTON.value))
        .onTrue(new InstantCommand(m_swerve::zeroGyro));

    // TODO Test
    new Trigger(() -> m_driver.getRawButton(Constants.Driver.LOCK_BUTTON.value))
        .onTrue(new InstantCommand(m_swerve::lock));

    // TODO Figure our what commands have to go here...
    new Trigger(() -> m_driver.getRawButton(Constants.Driver.PANIC_BUTTON.value))
        .onTrue(new SequentialCommandGroup(new InstantCommand(), new InstantCommand()));

    new Trigger(
            () ->
                m_manipulator.getRawAxis(Constants.Manipulator.CLOSE_AXIS.value)
                    > Constants.Manipulator.TRIGGER_THRESHOLD)
        .onTrue(new InstantCommand(m_claw::close));

    new Trigger(
            () ->
                m_manipulator.getRawAxis(Constants.Manipulator.OPEN_AXIS.value)
                    > Constants.Manipulator.TRIGGER_THRESHOLD)
        .onTrue(new InstantCommand(m_claw::open));

    new Trigger(
            () ->
                m_manipulator.getRawAxis(Constants.Manipulator.EXTEND_RETRACT_AXIS.value)
                    < -Constants.Manipulator.TRIGGER_THRESHOLD)
        .whileTrue(new SafeExtend(m_extensionController));

    new Trigger(
            () ->
                m_manipulator.getRawAxis(Constants.Manipulator.EXTEND_RETRACT_AXIS.value)
                    > Constants.Manipulator.TRIGGER_THRESHOLD)
        .whileTrue(new SafeRetract(m_extensionController));

    new Trigger(
            () ->
                m_manipulator.getRawAxis(Constants.Manipulator.RAISE_LOWER_AXIS.value)
                    < -Constants.Manipulator.TRIGGER_THRESHOLD)
        .whileTrue(new SafeRaise(m_rotationController));

    new Trigger(
            () ->
                m_manipulator.getRawAxis(Constants.Manipulator.RAISE_LOWER_AXIS.value)
                    > Constants.Manipulator.TRIGGER_THRESHOLD)
        .whileTrue(new SafeLower(m_rotationController));

    new Trigger(() -> m_manipulator.getRawButton(Constants.Manipulator.FLOOR_BUTTON.value))
        .whileTrue(new DumbRotate(m_rotationController, Constants.Arm.Angles.FLOOR));

    new Trigger(() -> m_manipulator.getRawButton(Constants.Manipulator.LEVEL_BUTTON.value))
        .whileTrue(new DumbRotate(m_rotationController, 0));

    new Trigger(() -> m_manipulator.getRawButton(Constants.Manipulator.TOP_BUTTON.value))
        .whileTrue(new DumbRotate(m_rotationController, -100));

    new Trigger(() -> m_manipulator.getRawButton(Constants.Manipulator.SUBSTATION_BUTTON.value))
        .whileTrue(new DumbRotate(m_rotationController, Constants.Arm.Angles.SUBSTATION));
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