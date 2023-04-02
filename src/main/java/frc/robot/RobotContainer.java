package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.auto.Autos;
import frc.robot.intake.SideIntake;
import frc.robot.superstructure.ArmState;
import frc.robot.superstructure.ExtensionController;
import frc.robot.superstructure.RollerClaw;
import frc.robot.superstructure.RotationController;
import frc.robot.swerve.AbsoluteDrive;
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

  private final Autos m_autos =
      new Autos(m_swerve, m_extensionController, m_rotationController, m_claw);

  private final CommandXboxController m_driver = new CommandXboxController(0);
  private final CommandXboxController m_manipulator = new CommandXboxController(1);

  private SendableChooser<Command> m_scoreChooser = new SendableChooser<Command>();
  private SendableChooser<Command> m_mobilityChooser = new SendableChooser<Command>();

  private final double DEADBAND = 0.5;

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
                    m_driver.getRawAxis(XboxController.Axis.kLeftY.value), DEADBAND),
            () ->
                MathUtil.applyDeadband(
                    m_driver.getRawAxis(XboxController.Axis.kLeftX.value), DEADBAND),
            () ->
                MathUtil.applyDeadband(
                    m_driver.getRawAxis(XboxController.Axis.kRightX.value), DEADBAND),
            () -> true, // Always drive field-oriented
            false,
            false);

    AbsoluteDrive absoluteDrive =
        new AbsoluteDrive(
            m_swerve,
            () ->
                MathUtil.applyDeadband(
                    m_driver.getRawAxis(XboxController.Axis.kLeftY.value), DEADBAND),
            () ->
                MathUtil.applyDeadband(
                    m_driver.getRawAxis(XboxController.Axis.kLeftX.value), DEADBAND),
            () ->
                MathUtil.applyDeadband(
                    m_driver.getRawAxis(XboxController.Axis.kRightX.value), DEADBAND),
            () ->
                MathUtil.applyDeadband(
                    m_driver.getRawAxis(XboxController.Axis.kRightY.value), DEADBAND),
            () -> m_driver.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.5);

    m_swerve.setDefaultCommand(absoluteDrive);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_driver.y().onTrue(new InstantCommand(m_swerve::zeroGyro));
    m_driver.x().onTrue(new InstantCommand(m_swerve::lock));

    m_manipulator
        .axisGreaterThan(XboxController.Axis.kLeftTrigger.value, DEADBAND)
        .onTrue(new InstantCommand(m_claw::intake))
        .onFalse(new InstantCommand(m_claw::hold));

    m_manipulator
        .axisGreaterThan(XboxController.Axis.kRightTrigger.value, DEADBAND)
        .onTrue(new InstantCommand(m_claw::outtake))
        .onFalse(new InstantCommand(m_claw::stop));

    m_manipulator
        .axisLessThan(XboxController.Axis.kRightY.value, -DEADBAND)
        .whileTrue(m_extensionController.extend());
    m_manipulator
        .axisGreaterThan(XboxController.Axis.kRightY.value, DEADBAND)
        .whileTrue(m_extensionController.retract());

    m_manipulator
        .axisLessThan(XboxController.Axis.kLeftY.value, -DEADBAND)
        .whileTrue(m_rotationController.raise());
    m_manipulator
        .axisGreaterThan(XboxController.Axis.kLeftY.value, DEADBAND)
        .whileTrue(m_rotationController.lower());

    m_manipulator.x().whileTrue(m_rotationController.rotateTo(ArmState.FLOOR));
    m_manipulator.a().whileTrue(m_rotationController.rotateTo(ArmState.STOWED));
    m_manipulator.y().whileTrue(m_rotationController.rotateTo(ArmState.TOP));
    m_manipulator.b().whileTrue(m_rotationController.rotateTo(ArmState.DOUBLE_SUBSTATION));

    m_manipulator
        .leftBumper()
        .onTrue(new InstantCommand(m_intake::intake))
        .onFalse(new InstantCommand(m_intake::stop));
    m_manipulator
        .rightBumper()
        .onTrue(new InstantCommand(m_intake::outtake))
        .onFalse(new InstantCommand(m_intake::stop));
  }

  private void configureTriggers() {}

  private void setupAutos() {
    m_scoreChooser.setDefaultOption("Top", m_autos.scoreTop());
    m_scoreChooser.addOption("Middle", m_autos.scoreMiddle());
    m_scoreChooser.addOption("Bottom", m_autos.scoreBottom());
    m_scoreChooser.addOption("Don't Score", new InstantCommand());
    SmartDashboard.putData("Score Position", m_scoreChooser);

    m_mobilityChooser.setDefaultOption("Yes", m_autos.mobility());
    m_mobilityChooser.addOption("No", new InstantCommand());
    SmartDashboard.putData("Mobility?", m_mobilityChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command scoreCommand = m_scoreChooser.getSelected();
    Command mobilityCommand = m_mobilityChooser.getSelected();

    return scoreCommand.andThen(mobilityCommand);
  }
}
