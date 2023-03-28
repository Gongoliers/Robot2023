package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.Autos;
import frc.robot.intake.SideIntake;
import frc.robot.superstructure.ExtensionController;
import frc.robot.superstructure.RollerClaw;
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
  private final RotationController m_rotationController = new RotationController();
  private final ExtensionController m_extensionController =
      new ExtensionController(m_rotationController);
  private final SideIntake m_intake = new SideIntake();
  private final RollerClaw m_claw = new RollerClaw();

  // Controllers
  private final XboxController m_driver = new XboxController(Constants.Driver.CONTROLLER_PORT);
  private final XboxController m_manipulator =
      new XboxController(Constants.Manipulator.CONTROLLER_PORT);

  private SendableChooser<Command> m_chooser = new SendableChooser<Command>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    setDefaultCommands();
    configureButtonBindings();
    configureTriggers();
    setupAutos();
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
        .onTrue(new InstantCommand(m_claw::intake))
        .onFalse(new InstantCommand(m_claw::hold));

    new Trigger(
            () ->
                m_manipulator.getRawAxis(Constants.Manipulator.OPEN_AXIS.value)
                    > Constants.Manipulator.TRIGGER_THRESHOLD)
        .onTrue(new InstantCommand(m_claw::outtake))
        .onFalse(new InstantCommand(m_claw::stop));

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
        .whileTrue(new DumbRotate(m_rotationController, -300));

    new Trigger(() -> m_manipulator.getRawButton(Constants.Manipulator.LEVEL_BUTTON.value))
        .whileTrue(new DumbRotate(m_rotationController, 0));

    new Trigger(() -> m_manipulator.getRawButton(Constants.Manipulator.TOP_BUTTON.value))
        .whileTrue(new DumbRotate(m_rotationController, -100));

    new Trigger(() -> m_manipulator.getRawButton(Constants.Manipulator.SUBSTATION_BUTTON.value))
        .whileTrue(new DumbRotate(m_rotationController, -115));

    new Trigger(() -> m_manipulator.getRawButton(XboxController.Button.kLeftBumper.value))
        .onTrue(new InstantCommand(m_intake::intake))
        .onFalse(new InstantCommand(m_intake::stop));
    new Trigger(() -> m_manipulator.getRawButton(XboxController.Button.kRightBumper.value))
        .onTrue(new InstantCommand(m_intake::outtake))
        .onFalse(new InstantCommand(m_intake::stop));
  }

  private void configureTriggers() {}

  private void setupAutos() {
    m_chooser.setDefaultOption(
        "Score Top", Autos.scoreTop(m_extensionController, m_rotationController, m_claw));
    m_chooser.addOption(
        "Score Middle", Autos.scoreMiddle(m_extensionController, m_rotationController, m_claw));
    m_chooser.addOption(
        "Score Bottom", Autos.scoreBottom(m_extensionController, m_rotationController, m_claw));
    m_chooser.addOption(
        "Score Top Backup",
        Autos.scoreTopBackup(m_extensionController, m_rotationController, m_claw, m_swerve));
    m_chooser.addOption(
        "Score Middle Backup",
        Autos.scoreMiddleBackup(m_extensionController, m_rotationController, m_claw, m_swerve));
    m_chooser.addOption(
        "Score Bottom Backup",
        Autos.scoreBottomBackup(m_extensionController, m_rotationController, m_claw, m_swerve));
    m_chooser.addOption(
        "Score Top ChargeStation",
        Autos.scoreTopChargeStation(m_extensionController, m_rotationController, m_claw, m_swerve));
    m_chooser.addOption(
        "Score Middle ChargeStation",
        Autos.scoreMiddleChargeStation(
            m_extensionController, m_rotationController, m_claw, m_swerve));
    m_chooser.addOption(
        "Score Bottom ChargeStation",
        Autos.scoreBottomChargeStation(
            m_extensionController, m_rotationController, m_claw, m_swerve));
    m_chooser.addOption("Do Nothing", new InstantCommand());

    SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
