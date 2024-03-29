package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
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
import frc.robot.swerve.TeleopDriveWithPrecision;
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

  private final Compressor m_compressor = new Compressor(30, PneumaticsModuleType.REVPH);

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
    
    TeleopDriveWithPrecision teleopDriveWithPrecision =
            new TeleopDriveWithPrecision(
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
                () -> m_driver.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.5,
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
                    -m_driver.getRawAxis(XboxController.Axis.kRightX.value), DEADBAND),
            () ->
                MathUtil.applyDeadband(
                    -m_driver.getRawAxis(XboxController.Axis.kRightY.value), DEADBAND),
            () -> m_driver.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.5);

    m_swerve.setDefaultCommand(teleopDriveWithPrecision);
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

    m_manipulator.x().whileTrue(m_rotationController.rotateTo(ArmState.DOUBLE_SUBSTATION));
    m_manipulator.a().whileTrue(m_rotationController.rotateTo(ArmState.STOWED));

    m_manipulator
        .leftBumper()
        .onTrue(new InstantCommand(m_intake::intake))
        .onFalse(new InstantCommand(m_intake::stop));
    m_manipulator
        .rightBumper()
        .onTrue(new InstantCommand(m_intake::outtake))
        .onFalse(new InstantCommand(m_intake::stop));

    m_manipulator
        .start()
        .onTrue(new InstantCommand(this::startCompressor))
        .onFalse(new InstantCommand(this::stopCompressor));
  }

  private void configureTriggers() {}

  private void setupAutos() {
    m_scoreChooser.setDefaultOption("Top (Cone)", m_autos.scoreTopCone());
    m_scoreChooser.addOption("Middle (Cone)", m_autos.scoreMiddleCone());
    m_scoreChooser.addOption("Bottom (Cone)", m_autos.scoreBottomCone());
    m_scoreChooser.setDefaultOption("Top (Cube)", m_autos.scoreTopCube());
    m_scoreChooser.addOption("Middle (Cube)", m_autos.scoreMiddleCube());
    m_scoreChooser.addOption("Bottom (Cube)", m_autos.scoreBottomCube());
    m_scoreChooser.addOption("Just Outtake", m_autos.outtake());
    m_scoreChooser.addOption("Don't Score", new InstantCommand());
    SmartDashboard.putData("Score Position", m_scoreChooser);

    m_mobilityChooser.setDefaultOption("Back Up", m_autos.mobility());
    m_mobilityChooser.addOption("Don't Move", new InstantCommand());
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

  public void startCompressor() {
    m_compressor.enableDigital();
  }

  public void stopCompressor() {
    m_compressor.disable();
  }
}
